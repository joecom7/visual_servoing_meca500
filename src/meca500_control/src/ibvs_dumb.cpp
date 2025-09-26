#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include "meca500_interfaces/srv/get_image_jacobian.hpp"
#include <Eigen/Dense>
#include <vector>

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

    this->declare_parameter<int>("cycle_frequency_hz", 1000);
    cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_no_effort", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          current_joint_positions_ = msg->position;
          received_joint_state_ = true;
        });

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/target_poses", 10, [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
          if (!msg->poses.empty())
          {
            target_u_ = msg->poses[0].position.x;
            target_v_ = msg->poses[0].position.y;
            target_depth_ = msg->poses[0].position.z;
            received_target_ = true;
          }
        });

    vel_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

    jacobian_client_ = this->create_client<GetJacobian>("get_jacobian");
    image_j_client_ = this->create_client<GetImageJacobian>("get_matrix");

    // Clients
    jacobian_client_ = this->create_client<GetJacobian>("get_jacobian");
    while (!jacobian_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for Jacobian service...");
    }

    image_j_client_ = this->create_client<GetImageJacobian>("get_matrix");
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

      image_j_client_->async_send_request(
          img_req, [this, J_robot](rclcpp::Client<GetImageJacobian>::SharedFuture img_future) {
            auto img_res = img_future.get();
            Eigen::MatrixXd J_img(img_res->rows, img_res->cols);
            Eigen::Map<Eigen::VectorXd>(J_img.data(), J_img.size()) =
                Eigen::Map<Eigen::VectorXd>(img_res->image_jacobian.data(), img_res->image_jacobian.size());

            // --- STEP 1: calcolo velocità end-effector in immagine ---
            Eigen::Vector2d e(0.0, -target_v_);
            Eigen::VectorXd v_e = k_p_ * J_img.completeOrthogonalDecomposition().pseudoInverse() * e;

            // Log per debug
            std::ostringstream oss_img;
            oss_img << "Required end-effector velocity (v_e): ";
            for (int i = 0; i < v_e.size(); ++i)
              oss_img << v_e[i] << " ";
            RCLCPP_INFO(this->get_logger(), "%s", oss_img.str().c_str());

            // --- STEP 2: calcolo velocità articolari ---
            Eigen::VectorXd q_dot = J_robot.completeOrthogonalDecomposition().pseudoInverse() * v_e;

            // Pubblico le velocità articolari
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

            // Log velocità articolari
            std::ostringstream oss_q;
            oss_q << "Publishing joint velocities: ";
            for (int i = 0; i < NUM_JOINTS; ++i)
              oss_q << vel_msg.name[i] << "=" << vel_msg.velocity[i] << " ";
            // RCLCPP_INFO(this->get_logger(), "%s", oss_q.str().c_str());
          });
    });
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr vel_pub_;
  rclcpp::Client<GetJacobian>::SharedPtr jacobian_client_;
  rclcpp::Client<GetImageJacobian>::SharedPtr image_j_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<double> current_joint_positions_{ NUM_JOINTS, 0.0 };
  bool received_joint_state_ = false;
  bool received_target_ = false;

  double target_u_ = 0, target_v_ = 0, target_depth_ = 1.0;
  double k_p_;
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
