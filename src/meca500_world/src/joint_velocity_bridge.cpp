#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

#define NUM_JOINTS 6
#define MECA500_VELOCITY_LIMITS                                                                                        \
  {                                                                                                                    \
    150.0, 150.0, 180.0, 300.0, 300.0, 500.0                                                                           \
  }

class JointVelocityBridge : public rclcpp::Node
{
public:
  JointVelocityBridge() : Node("joint_velocity_bridge")
  {
    // Velocità massime del Meca500 (rad/s)
    joint_velocity_limits_ = MECA500_VELOCITY_LIMITS;

    for (size_t i = 0; i < NUM_JOINTS; i++)
    {
      joint_velocity_limits_[i] = joint_velocity_limits_[i] * M_PI / 180.0;
    }

    // Publisher per ogni giunto
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      joint_publishers_.push_back(
          this->create_publisher<std_msgs::msg::Float64>("/joint" + std::to_string(i + 1) + "/cmd_vel", 10));
    }

    // Subscriber per i comandi di velocità
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_velocity_cmd", 10, std::bind(&JointVelocityBridge::joint_callback, this, std::placeholders::_1));
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.size() < NUM_JOINTS)
    {
      RCLCPP_WARN(this->get_logger(), "Received velocity command with fewer than %d joints", NUM_JOINTS);
      return;
    }

    // Calcola fattore di scala globale
    double scale = 1.0;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      double abs_v = std::fabs(msg->velocity[i]);
      if (abs_v > joint_velocity_limits_[i])
      {
        double s = joint_velocity_limits_[i] / abs_v;
        if (s < scale)
        {
          scale = s;  // prendiamo il fattore più restrittivo
        }
      }
    }

    // Applica scala a tutte le velocità
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      std_msgs::msg::Float64 vel_msg;
      vel_msg.data = msg->velocity[i] * scale;
      joint_publishers_[i]->publish(vel_msg);
    }
  }

  std::vector<double> joint_velocity_limits_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointVelocityBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
