#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include "meca500_interfaces/action/move_to_joint_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using GetJacobian = meca500_interfaces::srv::GetJacobian;
using MoveToJointPose = meca500_interfaces::action::MoveToJointPose;
using GoalHandleMoveToJointPose = rclcpp_action::ClientGoalHandle<MoveToJointPose>;

#define NUM_JOINTS 6
const int DEFAULT_CYCLE_FREQUENCY_HZ = 1000;
const double DEFAULT_CIRCLE_RADIUS = 0.05;
const double DEFAULT_CIRCLE_PERIOD = 3;
const double DEFAULT_MOTION_DURATION_S = 10.0;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("circle_yz_node");

  Eigen::Matrix3d camera_rotation_matrix;
  bool received_camera_rotation_matrix = false;

  node->declare_parameter<int>("cycle_frequency_hz", DEFAULT_CYCLE_FREQUENCY_HZ);
  int cycle_frequency_hz = node->get_parameter("cycle_frequency_hz").as_int();

  node->declare_parameter<double>("circle_radius", DEFAULT_CIRCLE_RADIUS);
  double circle_radius = node->get_parameter("circle_radius").as_double();

  node->declare_parameter<double>("circle_period_s", DEFAULT_CIRCLE_PERIOD);
  double circle_period_s = node->get_parameter("circle_period_s").as_double();

  node->declare_parameter<double>("motion_duration_s", DEFAULT_MOTION_DURATION_S);
  double motion_duration_s = node->get_parameter("motion_duration_s").as_double();

  // Parametro per home position
  node->declare_parameter<std::vector<double>>("home_position", {0, 0, 0, 0, 0, 0});
  std::vector<double> home_position = node->get_parameter("home_position").as_double_array();

  // Subscribers
  std::vector<double> current_joint_positions(NUM_JOINTS, 0.0);
  bool received_joint_state = false;

  auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [&current_joint_positions, &received_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < NUM_JOINTS; ++i)
          current_joint_positions[i] = msg->position[i];
        received_joint_state = true;
      });

  auto table_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/table_pose_relative_to_camera", 10,
      [&camera_rotation_matrix, &received_camera_rotation_matrix](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        Eigen::Quaterniond q_cam(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                 msg->pose.orientation.z);
        camera_rotation_matrix = q_cam.toRotationMatrix();
        received_camera_rotation_matrix = true;
      });

  // Publisher
  auto vel_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

  // Jacobian client
  auto jacobian_client = node->create_client<GetJacobian>("get_jacobian");
  while (!jacobian_client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(node->get_logger(), "Waiting for Jacobian service...");
  }

  // MoveToJointPose Action Client
  auto action_client = rclcpp_action::create_client<MoveToJointPose>(node, "/move_to_joint_pose");
  RCLCPP_INFO(node->get_logger(), "Waiting for /move_to_joint_pose action server...");
  if (!action_client->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "MoveToJointPose action server not available. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  // Send Home goal
  RCLCPP_INFO(node->get_logger(), "Sending robot to home position...");
  MoveToJointPose::Goal goal;
  goal.target_joints = home_position;

  bool goal_done = false;
  rclcpp_action::Client<MoveToJointPose>::SendGoalOptions options;
  options.goal_response_callback = [&node, &goal_done](GoalHandleMoveToJointPose::SharedPtr gh) {
    if (!gh)
      RCLCPP_ERROR(node->get_logger(), "Home goal was rejected by server");
    else
      RCLCPP_INFO(node->get_logger(), "Home goal accepted");
  };
  options.result_callback = [&node, &goal_done](const GoalHandleMoveToJointPose::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(node->get_logger(), "Reached home position successfully");
    else
      RCLCPP_ERROR(node->get_logger(), "Failed to reach home position");
    goal_done = true;
  };

  action_client->async_send_goal(goal, options);

  // Wait for homing to complete before continuing
  RCLCPP_INFO(node->get_logger(), "Waiting for homing to complete...");
  while (rclcpp::ok() && !goal_done)
  {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(), "Starting circular motion after homing.");
  RCLCPP_INFO(node->get_logger(), "Motion duration: %.2f seconds", motion_duration_s);

  // Main control loop
  rclcpp::WallRate rate(cycle_frequency_hz);
  auto t0 = node->get_clock()->now();
  auto motion_start_time = t0;
  bool motion_completed = false;

  while (rclcpp::ok() && !motion_completed)
  {
    rclcpp::spin_some(node);
    if (!received_joint_state || !received_camera_rotation_matrix)
      continue;

    double elapsed_time = (node->get_clock()->now() - motion_start_time).seconds();

    // Check if motion duration has been exceeded
    if (elapsed_time >= motion_duration_s)
    {
      RCLCPP_INFO(node->get_logger(), "Motion duration completed (%.2f seconds). Stopping.", elapsed_time);
      
      // Publish zero velocities to stop the robot
      sensor_msgs::msg::JointState stop_msg;
      stop_msg.header.stamp = node->get_clock()->now();
      stop_msg.name.resize(NUM_JOINTS);
      stop_msg.velocity.resize(NUM_JOINTS);
      
      for (int i = 0; i < NUM_JOINTS; ++i)
      {
        stop_msg.name[i] = "meca_axis_" + std::to_string(i + 1) + "_joint";
        stop_msg.velocity[i] = 0.0;
      }
      
      vel_pub->publish(stop_msg);
      motion_completed = true;
      break;
    }

    // Call Jacobian service
    auto request = std::make_shared<GetJacobian::Request>();
    auto result = jacobian_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::milliseconds(100)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node->get_logger(), "Failed to call Jacobian service");
      continue;
    }

    auto response = result.get();
    Eigen::MatrixXd J(response->rows, response->cols);
    Eigen::Map<Eigen::VectorXd>(J.data(), J.size()) =
        Eigen::Map<Eigen::VectorXd>(response->jacobian.data(), response->jacobian.size());

    Eigen::MatrixXd J_tool(6, NUM_JOINTS);
    J_tool.topRows(3) = camera_rotation_matrix * J.topRows(3);
    J_tool.bottomRows(3) = camera_rotation_matrix * J.bottomRows(3);

    // Desired circular trajectory (YZ-plane)
    double dt = (node->get_clock()->now() - t0).seconds();
    double omega = 2.0 * M_PI / circle_period_s;

    Eigen::Vector3d vel_xyz(-circle_radius * omega * sin(omega * dt), 0.0, circle_radius * omega * cos(omega * dt));
    Eigen::VectorXd vel6d(6);
    vel6d << vel_xyz, Eigen::Vector3d::Zero();

    Eigen::VectorXd q_dot = J_tool.completeOrthogonalDecomposition().pseudoInverse() * vel6d;

    sensor_msgs::msg::JointState vel_msg;
    vel_msg.header.stamp = node->get_clock()->now();
    vel_msg.name.resize(NUM_JOINTS);
    vel_msg.velocity.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      vel_msg.name[i] = "meca_axis_" + std::to_string(i + 1) + "_joint";
      vel_msg.velocity[i] = q_dot[i];
    }

    vel_pub->publish(vel_msg);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}