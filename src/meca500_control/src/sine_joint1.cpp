#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "math.h"

const int DEFAULT_CYCLE_FREQUENCY_HZ = 1000;
const double DEFAULT_SINE_WAVE_PERIOD_S = 5.0;
const double DEFAULT_SINE_WAVE_AMPLITUDE = 1.0;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("main");

  node->declare_parameter<int>("cycle_frequency_hz", DEFAULT_CYCLE_FREQUENCY_HZ);
  int cycle_frequency_hz = node->get_parameter("cycle_frequency_hz").as_int();

  node->declare_parameter<double>("sine_wave_period_s", DEFAULT_SINE_WAVE_PERIOD_S);
  double sine_wave_period_s = node->get_parameter("sine_wave_period_s").as_double();

  double sine_wave_w = M_PI * 2 / sine_wave_period_s;

  node->declare_parameter<double>("sine_wave_amplitude", DEFAULT_SINE_WAVE_AMPLITUDE);
  double sine_wave_amplitude = node->get_parameter("sine_wave_amplitude").as_double();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher =
      node->create_publisher<std_msgs::msg::Float64>("/joint1/cmd_vel", 10);

  auto t0 = node->get_clock()->now();
  bool initialized = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [node, &t0, &initialized](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
            if (!initialized && fabs(msg->position[0]) < 1e-4)
            {
              t0 = node->get_clock()->now();
            }
            else
            {
              initialized = true;
            }
          });

  rclcpp::WallRate loop_rate(cycle_frequency_hz);

  double derivative_amplitude = sine_wave_w*sine_wave_amplitude;

  while (rclcpp::ok())
  {
    rclcpp::Duration dt = node->get_clock()->now() - t0;

    auto message = std_msgs::msg::Float64();
    message.data = derivative_amplitude * cos(sine_wave_w * dt.seconds());

    publisher->publish(message);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}