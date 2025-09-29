#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "meca500_interfaces/srv/get_jacobian.hpp"
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using GetJacobian = meca500_interfaces::srv::GetJacobian;

#define NUM_JOINTS 6

const int DEFAULT_CYCLE_FREQUENCY_HZ = 1000;
const double DEFAULT_CIRCLE_RADIUS = 0.05; // 5 cm circle
const double DEFAULT_CIRCLE_PERIOD = 3;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("circle_yz_node");

    node->declare_parameter<int>("cycle_frequency_hz", DEFAULT_CYCLE_FREQUENCY_HZ);
    int cycle_frequency_hz = node->get_parameter("cycle_frequency_hz").as_int();

    node->declare_parameter<double>("circle_radius", DEFAULT_CIRCLE_RADIUS);
    double circle_radius = node->get_parameter("circle_radius").as_double();

    node->declare_parameter<double>("circle_period_s", DEFAULT_CIRCLE_PERIOD);
    double circle_period_s = node->get_parameter("circle_period_s").as_double();

    // Subscribers
    std::vector<double> current_joint_positions(NUM_JOINTS, 0.0);
    bool received_joint_state = false;

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&current_joint_positions, &received_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg){
            for (int i = 0; i < NUM_JOINTS; ++i)
                current_joint_positions[i] = msg->position[i];
            received_joint_state = true;
        });

    // Publisher: one standard JointState for velocities
    auto vel_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

    // Jacobian client
    auto jacobian_client = node->create_client<GetJacobian>("get_jacobian");
    while (!jacobian_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Jacobian service...");
    }

    rclcpp::WallRate rate(cycle_frequency_hz);
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
        double omega = 2.0 * M_PI / circle_period_s;

        Eigen::Vector3d vel_xyz(-circle_radius * omega * sin(omega*dt), 0, circle_radius * omega * cos(omega*dt));
        Eigen::VectorXd vel6d(NUM_JOINTS);
        vel6d << vel_xyz, Eigen::Vector3d::Zero(); // no rotation

        // Compute joint velocities via pseudoinverse
        Eigen::VectorXd q_dot = J.completeOrthogonalDecomposition().pseudoInverse() * vel6d;

        // Publish joint velocities as a single JointState message
        sensor_msgs::msg::JointState vel_msg;
        vel_msg.header.stamp = node->get_clock()->now();
        vel_msg.name.resize(NUM_JOINTS);
        vel_msg.velocity.resize(NUM_JOINTS);

        for (int i = 0; i < NUM_JOINTS; ++i) {
            vel_msg.name[i] = "meca_axis_" + std::to_string(i+1) + "_joint";
            vel_msg.velocity[i] = q_dot[i];
        }

        vel_pub->publish(vel_msg);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
