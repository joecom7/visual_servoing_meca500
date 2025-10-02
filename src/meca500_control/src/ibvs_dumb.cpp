#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include "meca500_interfaces/srv/get_image_jacobian.hpp"
#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

using GetJacobian = meca500_interfaces::srv::GetJacobian;
using GetImageJacobian = meca500_interfaces::srv::GetImageJacobian;

#define NUM_JOINTS 6

class IBVSNode : public rclcpp::Node
{
public:
    IBVSNode() : Node("ibvs_node")
    {
        this->declare_parameter<double>("k_p", 0.001);
        k_p_ = this->get_parameter("k_p").as_double();

        this->declare_parameter<double>("k_roll", 0.001);
        k_roll_ = this->get_parameter("k_roll").as_double();

        this->declare_parameter<double>("k_limit", 0.01);
        k_limit_ = this->get_parameter("k_limit").as_double();

        this->declare_parameter<double>("limit_threshold_deg", 15.0);
        limit_threshold_deg_ = this->get_parameter("limit_threshold_deg").as_double();

        this->declare_parameter<int>("cycle_frequency_hz", 1000);
        cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();

        // Declare joint limit parameters (in degrees)
        this->declare_parameter<std::vector<double>>("joint_limits_lower", 
            std::vector<double>{-175.0, -70.0, -135.0, -170.0, -115.0, -36000.0});
        this->declare_parameter<std::vector<double>>("joint_limits_upper", 
            std::vector<double>{175.0, 90.0, 70.0, 170.0, 115.0, 36000.0});

        // Initialize joint limits (convert to radians)
        auto limits_lower_deg = this->get_parameter("joint_limits_lower").as_double_array();
        auto limits_upper_deg = this->get_parameter("joint_limits_upper").as_double_array();
        
        if (limits_lower_deg.size() != NUM_JOINTS || limits_upper_deg.size() != NUM_JOINTS) {
            RCLCPP_ERROR(this->get_logger(), "Joint limits must have %d values!", NUM_JOINTS);
            throw std::runtime_error("Invalid joint limits size");
        }
        
        joint_limits_lower_.resize(NUM_JOINTS);
        joint_limits_upper_.resize(NUM_JOINTS);
        
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_limits_lower_[i] = limits_lower_deg[i] * M_PI / 180.0;
            joint_limits_upper_[i] = limits_upper_deg[i] * M_PI / 180.0;
        }
        
        RCLCPP_INFO(this->get_logger(), "Joint limits loaded (deg): Lower=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], Upper=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            limits_lower_deg[0], limits_lower_deg[1], limits_lower_deg[2], limits_lower_deg[3], limits_lower_deg[4], limits_lower_deg[5],
            limits_upper_deg[0], limits_upper_deg[1], limits_upper_deg[2], limits_upper_deg[3], limits_upper_deg[4], limits_upper_deg[5]);

        // Service per abilitare/disabilitare il controllo
        control_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/ibvs",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
                control_enabled_ = request->data;
                response->success = true;
                response->message = control_enabled_ ? "IBVS control enabled" : "IBVS control disabled";
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            });

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                current_joint_positions_ = msg->position;
                received_joint_state_ = true;
            });

        target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/filtered_target_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                target_u_ = msg->position.x;
                target_v_ = msg->position.y;
                target_depth_ = msg->position.z;
                received_target_ = true;
            });

        camera_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/camera_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                Eigen::Quaterniond q_cam(msg->pose.orientation.w,
                                        msg->pose.orientation.x,
                                        msg->pose.orientation.y,
                                        msg->pose.orientation.z);
                Eigen::Matrix3d R_cam_world = q_cam.toRotationMatrix();
                Eigen::Vector3d Z_wr = R_cam_world.col(2);
                Eigen::Vector3d proj_YZ(0, Z_wr.y(), Z_wr.z());
                roll_error = std::atan2(proj_YZ.y(), proj_YZ.z());
            });

        vel_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

        // Clients
        jacobian_client_ = this->create_client<GetJacobian>("/get_jacobian");
        while (!jacobian_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Jacobian service...");
        }

        image_j_client_ = this->create_client<GetImageJacobian>("/get_image_jacobian");
        while (!image_j_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Image Jacobian service...");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / cycle_frequency_hz_),
            std::bind(&IBVSNode::control_loop, this));
    }

private:
    // Compute joint limit avoidance velocity and task Jacobian
    void compute_limit_avoidance(Eigen::VectorXd& q_dot_limit, Eigen::MatrixXd& J_limit)
    {
        q_dot_limit = Eigen::VectorXd::Zero(NUM_JOINTS);
        std::vector<double> weights(NUM_JOINTS, 0.0);
        
        double threshold_rad = limit_threshold_deg_ * M_PI / 180.0;
        
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double q = current_joint_positions_[i];
            double q_lower = joint_limits_lower_[i];
            double q_upper = joint_limits_upper_[i];
            double q_mid = (q_lower + q_upper) / 2.0;
            double q_range = q_upper - q_lower;
            
            // Distance from lower and upper limits
            double dist_lower = q - q_lower;
            double dist_upper = q_upper - q;
            
            // Repulsive potential field
            double repulsion = 0.0;
            double activation = 0.0;
            
            // Near lower limit
            if (dist_lower < threshold_rad) {
                double ratio = dist_lower / threshold_rad;
                // Smooth activation: 1 at limit, 0 at threshold
                activation = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio); // smoothstep
                repulsion = activation; // positive velocity (away from lower limit)
                weights[i] = activation;
            }
            // Near upper limit
            else if (dist_upper < threshold_rad) {
                double ratio = dist_upper / threshold_rad;
                activation = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio); // smoothstep
                repulsion = -activation; // negative velocity (away from upper limit)
                weights[i] = activation;
            }
            
            q_dot_limit[i] = repulsion;
        }
        
        // Create weighted identity Jacobian
        // Active joints have weight > 0, inactive joints have weight = 0
        J_limit = Eigen::MatrixXd::Zero(NUM_JOINTS, NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            J_limit(i, i) = weights[i];
        }
        
        // Remove zero rows (inactive joints) for proper null space computation
        int active_count = 0;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (weights[i] > 1e-6) active_count++;
        }
        
        if (active_count > 0) {
            Eigen::MatrixXd J_active(active_count, NUM_JOINTS);
            Eigen::VectorXd q_dot_active(active_count);
            int row = 0;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                if (weights[i] > 1e-6) {
                    J_active.row(row) = J_limit.row(i);
                    q_dot_active[row] = q_dot_limit[i];
                    row++;
                }
            }
            J_limit = J_active;
            q_dot_limit = q_dot_active;
        } else {
            // No active constraints: return zero-row Jacobian
            J_limit = Eigen::MatrixXd::Zero(0, NUM_JOINTS);
            q_dot_limit = Eigen::VectorXd::Zero(0);
        }
    }

    void control_loop()
    {
        if (!control_enabled_)
            return;

        if (!received_joint_state_ || !received_target_)
            return;

        // Call robot Jacobian asynchronously
        auto jac_req = std::make_shared<GetJacobian::Request>();
        jacobian_client_->async_send_request(jac_req,
            [this](rclcpp::Client<GetJacobian>::SharedFuture future) {
                auto jac_res = future.get();
                Eigen::MatrixXd J_robot(jac_res->rows, jac_res->cols);
                Eigen::Map<Eigen::VectorXd>(J_robot.data(), J_robot.size()) =
                    Eigen::Map<Eigen::VectorXd>(jac_res->jacobian.data(), jac_res->jacobian.size());

                // Call image Jacobian asynchronously
                auto img_req = std::make_shared<GetImageJacobian::Request>();
                img_req->u = target_u_;
                img_req->v = target_v_;
                img_req->depth = target_depth_;

                image_j_client_->async_send_request(img_req,
                    [this, J_robot](rclcpp::Client<GetImageJacobian>::SharedFuture img_future) {
                        auto img_res = img_future.get();
                        Eigen::MatrixXd J_img(img_res->rows, img_res->cols);
                        Eigen::Map<Eigen::VectorXd>(J_img.data(), J_img.size()) =
                            Eigen::Map<Eigen::VectorXd>(img_res->image_jacobian.data(),
                                                       img_res->image_jacobian.size());

                        // === TASK 1: JOINT LIMIT AVOIDANCE (HIGHEST PRIORITY) ===
                        Eigen::VectorXd q_dot_limit;
                        Eigen::MatrixXd J_limit;
                        compute_limit_avoidance(q_dot_limit, J_limit);
                        
                        Eigen::VectorXd q_dot_limit_task = Eigen::VectorXd::Zero(NUM_JOINTS);
                        Eigen::MatrixXd P_limit = Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS);
                        
                        if (J_limit.rows() > 0) {
                            q_dot_limit_task = k_limit_ * J_limit.completeOrthogonalDecomposition().pseudoInverse() * q_dot_limit;
                            P_limit = Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS) -
                                     J_limit.completeOrthogonalDecomposition().pseudoInverse() * J_limit;
                        }

                        // === TASK 2: ROLL CONTROL (SECONDARY) ===
                        Eigen::MatrixXd J_roll = J_robot.row(3); // 1x6
                        Eigen::VectorXd q_dot_roll = -k_roll_ * 
                            J_roll.completeOrthogonalDecomposition().pseudoInverse() * roll_error;
                        
                        // Project through limit avoidance null space
                        q_dot_roll = P_limit * q_dot_roll;
                        
                        // Null space projector for both limit and roll
                        Eigen::MatrixXd P_roll = P_limit * 
                            (Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS) - 
                             J_roll.completeOrthogonalDecomposition().pseudoInverse() * J_roll);

                        // === TASK 3: IBVS (TERTIARY) ===
                        Eigen::Vector2d e(target_u_, target_v_);
                        Eigen::MatrixXd J_full = J_img * J_robot;
                        Eigen::VectorXd q_dot_ibvs = k_p_ * 
                            J_full.completeOrthogonalDecomposition().pseudoInverse() * e;
                        
                        // Project through both limit and roll null spaces
                        q_dot_ibvs = P_roll * q_dot_ibvs;

                        // === TOTAL VELOCITY ===
                        Eigen::VectorXd q_dot = q_dot_limit_task + q_dot_roll + q_dot_ibvs;

                        // Publish
                        sensor_msgs::msg::JointState vel_msg;
                        vel_msg.header.stamp = this->get_clock()->now();
                        vel_msg.name.resize(NUM_JOINTS);
                        vel_msg.velocity.resize(NUM_JOINTS);
                        for (int i = 0; i < NUM_JOINTS; ++i) {
                            vel_msg.name[i] = "meca_axis_" + std::to_string(i + 1) + "_joint";
                            vel_msg.velocity[i] = q_dot[i];
                        }
                        vel_pub_->publish(vel_msg);
                    });
            });
    }

    // Members
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr vel_pub_;
    rclcpp::Client<GetJacobian>::SharedPtr jacobian_client_;
    rclcpp::Client<GetImageJacobian>::SharedPtr image_j_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> current_joint_positions_{NUM_JOINTS, 0.0};
    std::vector<double> joint_limits_lower_;
    std::vector<double> joint_limits_upper_;
    
    bool received_joint_state_ = false;
    bool received_target_ = false;
    bool control_enabled_ = false;
    
    double roll_error = 0.0;
    double target_u_ = 0, target_v_ = 0, target_depth_ = 1.0;
    double k_p_;
    double k_roll_;
    double k_limit_;
    double limit_threshold_deg_;
    int cycle_frequency_hz_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IBVSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}