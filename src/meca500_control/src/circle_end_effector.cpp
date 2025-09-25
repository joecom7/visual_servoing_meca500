#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using GetJacobian = meca500_interfaces::srv::GetJacobian;

const double RADIUS = 0.05; // 5 cm circle
const double FREQUENCY = 0.2; // Hz
const int NUM_JOINTS = 6; // Adjust to your robot

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("circle_yz_node");

    // Subscribers
    std::vector<double> current_joint_positions(NUM_JOINTS, 0.0);
    bool received_joint_state = false;

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_no_effort", 10,
        [&current_joint_positions, &received_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg){
            for (int i = 0; i < NUM_JOINTS; ++i)
                current_joint_positions[i] = msg->position[i];
            received_joint_state = true;
        });

    // Publishers: one per joint
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_publishers[i] = node->create_publisher<std_msgs::msg::Float64>("/joint" + std::to_string(i+1) + "/cmd_vel", 10);
    }

    // Jacobian client
    auto jacobian_client = node->create_client<GetJacobian>("get_jacobian");
    while (!jacobian_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Jacobian service...");
    }

    rclcpp::WallRate rate(100); // 100 Hz
    auto t0 = node->get_clock()->now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (!received_joint_state) continue;

        // Call Jacobian service
        auto request = std::make_shared<GetJacobian::Request>();
        auto result = jacobian_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result, std::chrono::milliseconds(100)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node->get_logger(), "Failed to call Jacobian service");
            continue;
        }

        auto response = result.get();
        Eigen::MatrixXd J(response->rows, response->cols);
        Eigen::Map<Eigen::VectorXd>(J.data(), J.size()) = Eigen::Map<Eigen::VectorXd>(response->jacobian.data(), response->jacobian.size());

        // Compute desired end-effector velocity for YZ circle
        double dt = (node->get_clock()->now() - t0).seconds();
        double omega = 2.0 * M_PI * FREQUENCY;

        Eigen::Vector3d vel_xyz( -RADIUS * omega * sin(omega*dt),0, RADIUS * omega * cos(omega*dt));
        Eigen::VectorXd vel6d(6);
        vel6d << vel_xyz, Eigen::Vector3d::Zero(); // no rotation

        // Compute joint velocities via pseudoinverse
        Eigen::VectorXd q_dot = J.completeOrthogonalDecomposition().pseudoInverse() * vel6d;

        // Publish each joint velocity on its topic
        for (int i = 0; i < NUM_JOINTS; ++i) {
            std_msgs::msg::Float64 msg;
            msg.data = q_dot[i];
            joint_publishers[i]->publish(msg);
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
