#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <limits>
#include <cmath>

using std::placeholders::_1;

class PoseFilter : public rclcpp::Node
{
public:
    PoseFilter() : Node("pose_filter")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/target_poses", 10, std::bind(&PoseFilter::poseArrayCallback, this, _1));

        pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/filtered_target_pose", 10);

        last_pose_ = nullptr;
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No poses received.");
            return;
        }

        // If no last pose, just take the first one
        if (!last_pose_) {
            last_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[0]);
            pub_->publish(*last_pose_);
            return;
        }

        // Find the closest pose to the last followed one
        double min_dist = std::numeric_limits<double>::max();
        geometry_msgs::msg::Pose closest_pose;

        for (const auto &pose : msg->poses) {
            double dx = pose.position.x - last_pose_->position.x;
            double dy = pose.position.y - last_pose_->position.y;
            double dz = pose.position.z - last_pose_->position.z;
            double dist = dx*dx + dy*dy + dz*dz;  // squared distance

            if (dist < min_dist) {
                min_dist = dist;
                closest_pose = pose;
            }
        }

        last_pose_ = std::make_shared<geometry_msgs::msg::Pose>(closest_pose);
        pub_->publish(*last_pose_);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    std::shared_ptr<geometry_msgs::msg::Pose> last_pose_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFilter>());
    rclcpp::shutdown();
    return 0;
}
