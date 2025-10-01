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

    this->declare_parameter<int>("cycle_frequency_hz", 1000);
    cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();

    // Service per abilitare/disabilitare il controllo
    control_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/ibvs", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                        std_srvs::srv::SetBool::Response::SharedPtr response) {
          control_enabled_ = request->data;
          response->success = true;
          response->message = control_enabled_ ? "IBVS control enabled" : "IBVS control disabled";
          RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        });

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          current_joint_positions_ = msg->position;
          received_joint_state_ = true;
        });

    // 🔥 QUI: invece di PoseArray leggiamo direttamente Pose
    target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/filtered_target_pose", 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
          target_u_ = msg->position.x;
          target_v_ = msg->position.y;
          target_depth_ = msg->position.z;
          received_target_ = true;
        });

    // Subscribe to camera pose
    camera_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/camera_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          // Convert quaternion to roll
          // Quaternion camera → world
          Eigen::Quaterniond q_cam(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                   msg->pose.orientation.z);

          // Convert to rotation matrix
          Eigen::Matrix3d R_cam_world = q_cam.toRotationMatrix();

          // Z-axis of WRF (same as camera) in world frame
          Eigen::Vector3d Z_wr = R_cam_world.col(2);  // terza colonna

          // RCLCPP_INFO(this->get_logger(), "Z_wr: [%f, %f, %f]", Z_wr.x(), Z_wr.y(), Z_wr.z());

          // Project Z on TRF YZ plane
          Eigen::Vector3d proj_YZ(0, Z_wr.y(), Z_wr.z());

          // Roll error needed to align Z vertical in TRF
          roll_error = std::atan2(proj_YZ.y(), proj_YZ.z());
        });

    vel_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

    // Clients
    jacobian_client_ = this->create_client<GetJacobian>("/get_jacobian");
    while (!jacobian_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for Jacobian service...");
    }

    image_j_client_ = this->create_client<GetImageJacobian>("/get_image_jacobian");
    while (!image_j_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for Image Jacobian service...");
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cycle_frequency_hz_),
                                     std::bind(&IBVSNode::control_loop, this));
  }

private:
  void control_loop()
  {
    if (!control_enabled_)  // skip se disabilitato
      return;
    if (!received_joint_state_ || !received_target_)
      return;

    // Call robot Jacobian asynchronously
    auto jac_req = std::make_shared<GetJacobian::Request>();
    jacobian_client_->async_send_request(jac_req, [this](rclcpp::Client<GetJacobian>::SharedFuture future) {
      auto jac_res = future.get();
      Eigen::MatrixXd J_robot(jac_res->rows, jac_res->cols);
      Eigen::Map<Eigen::VectorXd>(J_robot.data(), J_robot.size()) =
          Eigen::Map<Eigen::VectorXd>(jac_res->jacobian.data(), jac_res->jacobian.size());

      // Call image Jacobian asynchronously
      auto img_req = std::make_shared<GetImageJacobian::Request>();
      img_req->u = target_u_;
      img_req->v = target_v_;
      img_req->depth = target_depth_;

      image_j_client_->async_send_request(img_req, [this, J_robot](
                                                       rclcpp::Client<GetImageJacobian>::SharedFuture img_future) {
        auto img_res = img_future.get();
        Eigen::MatrixXd J_img(img_res->rows, img_res->cols);
        Eigen::Map<Eigen::VectorXd>(J_img.data(), J_img.size()) =
            Eigen::Map<Eigen::VectorXd>(img_res->image_jacobian.data(), img_res->image_jacobian.size());

        Eigen::MatrixXd J_img_mod(J_img.rows(), J_img.cols() - 1);
        J_img_mod << J_img.leftCols(3), J_img.rightCols(J_img.cols() - 4);

        Eigen::MatrixXd J_robot_mod(J_robot.rows() - 1, J_robot.cols());
        J_robot_mod << J_robot.topRows(3), J_robot.bottomRows(J_robot.rows() - 4);

        Eigen::MatrixXd J_roll = J_robot.row(3);  // 1x6
        Eigen::VectorXd q_dot_roll = -k_roll_ * J_roll.completeOrthogonalDecomposition().pseudoInverse() * roll_error;

        // Proiettore nullo
        Eigen::MatrixXd P_roll = Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS) -
                                 J_roll.completeOrthogonalDecomposition().pseudoInverse() * J_roll;

        // q_dot IBVS senza influenzare il roll
        Eigen::Vector2d e(target_u_, target_v_);
        // RCLCPP_INFO(this->get_logger(), "roll error: %f", roll_error);

        Eigen::MatrixXd J_full = J_img * J_robot;

        Eigen::VectorXd q_dot_ibvs = k_p_ * J_full.completeOrthogonalDecomposition().pseudoInverse() * e;
        q_dot_ibvs = P_roll * q_dot_ibvs;

        // Somma delle velocità
        Eigen::VectorXd q_dot = q_dot_roll + q_dot_ibvs;

        sensor_msgs::msg::JointState vel_msg;
        vel_msg.header.stamp = this->get_clock()->now();
        vel_msg.name.resize(NUM_JOINTS);
        vel_msg.velocity.resize(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
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
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<double> current_joint_positions_{ NUM_JOINTS, 0.0 };
  bool received_joint_state_ = false;
  bool received_target_ = false;
  bool control_enabled_ = false;

  double roll_error = 0.0;

  double target_u_ = 0, target_v_ = 0, target_depth_ = 1.0;
  double k_p_;
  double k_roll_;
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
