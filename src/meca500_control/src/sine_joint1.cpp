#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "math.h"

const int DEFAULT_CYCLE_FREQUENCY_HZ = 1000;
const double DEFAULT_SINE_WAVE_PERIOD_S = 5.0;
const double DEFAULT_SINE_WAVE_AMPLITUDE = 1.0;

#define NUM_JOINTS 6

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sine_velocity_node");

  node->declare_parameter<int>("cycle_frequency_hz", DEFAULT_CYCLE_FREQUENCY_HZ);
  int cycle_frequency_hz = node->get_parameter("cycle_frequency_hz").as_int();

  node->declare_parameter<double>("sine_wave_period_s", DEFAULT_SINE_WAVE_PERIOD_S);
  double sine_wave_period_s = node->get_parameter("sine_wave_period_s").as_double();

  double sine_wave_w = 2.0 * M_PI / sine_wave_period_s;

  node->declare_parameter<double>("sine_wave_amplitude", DEFAULT_SINE_WAVE_AMPLITUDE);
  double sine_wave_amplitude = node->get_parameter("sine_wave_amplitude").as_double();

  // Publisher for joint velocities
  auto vel_pub = node->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_velocity_cmd", 10);

  auto t0 = node->get_clock()->now();
  bool initialized = false;

  // Watching /joint_states to sync the start time
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [node, &t0, &initialized](sensor_msgs::msg::JointState::UniquePtr msg) {
        if (!initialized && std::fabs(msg->position[0]) < 1e-4)
        {
          t0 = node->get_clock()->now();
        }
        else
        {
          initialized = true;
        }
      });

  rclcpp::WallRate loop_rate(cycle_frequency_hz);

  double derivative_amplitude = sine_wave_w * sine_wave_amplitude;

  while (rclcpp::ok())
  {
    rclcpp::Duration dt = node->get_clock()->now() - t0;

    // Create JointState message
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->get_clock()->now();
    msg.name.resize(NUM_JOINTS);
    msg.velocity.resize(NUM_JOINTS);

    // Joint names consistent with your MoveToJointPose code
    for (int i = 0; i < NUM_JOINTS; ++i)
      msg.name[i] = "meca_axis_" + std::to_string(i + 1) + "_joint";

    // Apply sine wave derivative ONLY to joint 1
    msg.velocity[0] = derivative_amplitude * std::cos(sine_wave_w * dt.seconds());

    // Other joints = 0
    for (int i = 1; i < NUM_JOINTS; ++i)
      msg.velocity[i] = 0.0;

    vel_pub->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
