#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "meca500_interfaces/action/move_to_joint_pose.hpp"
#include <vector>
#include <Eigen/Dense>

using MoveToJointPose = meca500_interfaces::action::MoveToJointPose;
using GoalHandleMoveToJointPose = rclcpp_action::ServerGoalHandle<MoveToJointPose>;

#define NUM_JOINTS 6

class JointPoseActionServer : public rclcpp::Node
{
public:
  JointPoseActionServer() : Node("joint_pose_action_server")
  {
    this->declare_parameter<double>("k_p", 0.5);
    k_p_ = this->get_parameter("k_p").as_double();

    this->declare_parameter<double>("joint_norm_tolerance", 0.01);
    joint_norm_tolerance_ = this->get_parameter("joint_norm_tolerance").as_double();

    this->declare_parameter<int>("cycle_frequency_hz", 1000);
    cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          current_joint_positions_ = msg->position;
          received_joint_state_ = true;
        });

    vel_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

    action_server_ = rclcpp_action::create_server<MoveToJointPose>(
        this, "/move_to_joint_pose",
        std::bind(&JointPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointPoseActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&JointPoseActionServer::handle_accepted, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1e+3 / cycle_frequency_hz_)),
                                     std::bind(&JointPoseActionServer::control_loop, this));
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const MoveToJointPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received new joint goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToJointPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToJointPose> goal_handle)
  {
    current_goal_ = goal_handle;
    target_joint_positions_ = current_goal_->get_goal()->target_joints;
    RCLCPP_INFO(this->get_logger(), "Target joint positions set");
  }

  void control_loop()
  {
    if (!received_joint_state_ || target_joint_positions_.empty() || !current_goal_)
      return;

    Eigen::VectorXd q_current(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i)
      q_current[i] = current_joint_positions_[i];

    Eigen::VectorXd q_target(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i)
      q_target[i] = target_joint_positions_[i];

    Eigen::VectorXd q_dot = k_p_ * (q_target - q_current);

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

    // Feedback
    auto feedback = std::make_shared<MoveToJointPose::Feedback>();
    feedback->current_joints = current_joint_positions_;
    current_goal_->publish_feedback(feedback);

    if ((q_target - q_current).norm() < joint_norm_tolerance_)
    {
      sensor_msgs::msg::JointState stop_msg;
      stop_msg.header.stamp = this->get_clock()->now();
      stop_msg.name.resize(NUM_JOINTS);
      stop_msg.velocity.resize(NUM_JOINTS, 0.0);
      for (int i = 0; i < NUM_JOINTS; ++i)
      {
        stop_msg.name[i] = "meca_axis_" + std::to_string(i + 1) + "_joint";
      }
      vel_pub_->publish(stop_msg);

      // Segnala al client che il goal è raggiunto
      auto result = std::make_shared<MoveToJointPose::Result>();
      result->success = true;
      current_goal_->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal reached and velocities set to 0");

      current_goal_.reset();
      target_joint_positions_.clear();
    }
  }

  // Members
  double k_p_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr vel_pub_;
  rclcpp_action::Server<MoveToJointPose>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleMoveToJointPose> current_goal_;
  rclcpp::TimerBase::SharedPtr timer_;

  int cycle_frequency_hz_;
  double joint_norm_tolerance_;

  std::vector<double> current_joint_positions_{ NUM_JOINTS, 0.0 };
  std::vector<double> target_joint_positions_;
  bool received_joint_state_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPoseActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
