#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>
#include <vector>

#define NUM_JOINTS 6

class JointVelocityBridge : public rclcpp::Node
{
public:
  JointVelocityBridge() : Node("joint_velocity_bridge_no_clamp")
  {
    // Publisher toward Xenomai controller
    xeno_pub_ = this->create_publisher<xenopkg_interfaces::msg::Float32Header>(
      "/cmd_vel_xeno", 10);

    // Subscriber from standard velocity topic
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_velocity_cmd", 10,
      std::bind(&JointVelocityBridge::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JointVelocityBridge (no clamping) started.");
  }

private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.size() < NUM_JOINTS)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Received JointState with only %zu velocity entries (expected %d)",
        msg->velocity.size(), NUM_JOINTS);
      return;
    }

    xenopkg_interfaces::msg::Float32Header out;
    out.data.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; ++i)
      out.data[i] = static_cast<float>(msg->velocity[i]);

    xeno_pub_->publish(out);
  }

  rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr xeno_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointVelocityBridge>());
  rclcpp::shutdown();
  return 0;
}
