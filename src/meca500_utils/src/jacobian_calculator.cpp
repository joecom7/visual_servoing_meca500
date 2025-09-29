#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using GetJacobian = meca500_interfaces::srv::GetJacobian;

class JacobianServiceNode : public rclcpp::Node
{
private:
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_sub_;
  rclcpp::Service<GetJacobian>::SharedPtr jacobian_service_;

  // MoveIt
  std::shared_ptr<rclcpp::Node> robot_loader_node_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::RobotStatePtr kinematic_state_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_joint_positions_;
  bool received_joint_state_ = false;

  const std::string ROBOT_MODEL_GROUP = "meca500_arm";  // adjust to your MoveIt SRDF group
  const std::string EE_LINK = "D435i_camera_link";      // end-effector link

public:
  JacobianServiceNode() : Node("jacobian_service_node")
  {
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          current_joint_positions_ = msg->position;
          received_joint_state_ = true;
        });

    // Create service
    jacobian_service_ =
        this->create_service<GetJacobian>("/get_jacobian", [this](const std::shared_ptr<GetJacobian::Request> request,
                                                                  std::shared_ptr<GetJacobian::Response> response) {
          (void)request;
          if (!received_joint_state_)
          {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet!");
            return;
          }

          Eigen::MatrixXd jacobian;
          kinematic_state_->setVariablePositions(current_joint_positions_);
          kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(EE_LINK),
                                        Eigen::Vector3d::Zero(), jacobian, false);

          response->rows = jacobian.rows();
          response->cols = jacobian.cols();
          response->jacobian.resize(jacobian.size());
          Eigen::Map<Eigen::VectorXd>(response->jacobian.data(), jacobian.size()) =
              Eigen::Map<Eigen::VectorXd>(jacobian.data(), jacobian.size());
        });

    setup_moveit();
    RCLCPP_INFO(this->get_logger(), "Jacobian service node ready!");
  }

  void setup_moveit()
  {
    // Separate node for RobotModelLoader
    robot_loader_node_ = std::make_shared<rclcpp::Node>(
        "robot_model_loader_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_loader_node_);
    auto kinematic_model = robot_model_loader_->getModel();

    joint_model_group_ = kinematic_model->getJointModelGroup(ROBOT_MODEL_GROUP);
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);

    joint_names_ = joint_model_group_->getVariableNames();
    current_joint_positions_.resize(joint_names_.size(), 0.0);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JacobianServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
