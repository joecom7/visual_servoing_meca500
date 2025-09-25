#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>

#define NUM_JOINTS 6

class JointVelocityBridge : public rclcpp::Node
{
public:
    JointVelocityBridge() : Node("joint_velocity_bridge")
    {
        // Publishers for each joint
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_publishers_.push_back(
                this->create_publisher<std_msgs::msg::Float64>("/joint" + std::to_string(i+1) + "/cmd_vel", 10)
            );
        }

        // Subscriber for standard joint velocity command
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_velocity_cmd", 10,
            std::bind(&JointVelocityBridge::joint_callback, this, std::placeholders::_1)
        );
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->velocity.size() < NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(), "Received velocity command with fewer than %d joints", NUM_JOINTS);
            return;
        }

        for (int i = 0; i < NUM_JOINTS; ++i) {
            std_msgs::msg::Float64 vel_msg;
            vel_msg.data = msg->velocity[i];
            joint_publishers_[i]->publish(vel_msg);
        }
    }

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointVelocityBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}