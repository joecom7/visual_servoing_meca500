#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

const int DEFAULT_CYCLE_FREQUENCY_HZ = 1000;

class CameraPosePublisher : public rclcpp::Node
{
public:
  CameraPosePublisher() : Node("camera_pose_publisher")
  {
    // Publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);

    // TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter<int>("cycle_frequency_hz", DEFAULT_CYCLE_FREQUENCY_HZ);
    int cycle_frequency_hz = this->get_parameter("cycle_frequency_hz").as_int();

    // Timer to publish at 10 Hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1e+3 / float(cycle_frequency_hz))),
                                     std::bind(&CameraPosePublisher::publish_camera_pose, this));
  }

private:
  void publish_camera_pose()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer_->lookupTransform("D435i_camera_link", "world", tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "D435i_camera_link";
      pose_msg.pose.position.x = transformStamped.transform.translation.x;
      pose_msg.pose.position.y = transformStamped.transform.translation.y;
      pose_msg.pose.position.z = transformStamped.transform.translation.z;
      pose_msg.pose.orientation = transformStamped.transform.rotation;

      pose_pub_->publish(pose_msg);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
