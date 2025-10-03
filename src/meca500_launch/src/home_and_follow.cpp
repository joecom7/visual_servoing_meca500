#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "meca500_interfaces/action/move_to_joint_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <chrono>

using MoveToJointPose = meca500_interfaces::action::MoveToJointPose;
using GoalHandleMoveToJointPose = rclcpp_action::ClientGoalHandle<MoveToJointPose>;

class HomeAndStartIBVS : public rclcpp::Node
{
public:
  HomeAndStartIBVS() : Node("home_and_start_ibvs")
  {
    this->declare_parameter<int>("cycle_frequency_hz", 1000);
    this->declare_parameter<double>("no_target_timeout_ms", 500.0);
    this->declare_parameter<std::vector<double>>("home_position", { 0, 0, 0, 0, 0, 0 });
    this->declare_parameter<std::vector<double>>("initial_position", { 1.57, 0.0, 0.0, 0.0, 0.0, 0.0 });

    cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();
    no_target_timeout_ms_ = this->get_parameter("no_target_timeout_ms").as_double();
    home_position_ = this->get_parameter("home_position").as_double_array();
    initial_position_ = this->get_parameter("initial_position").as_double_array();

    action_client_ = rclcpp_action::create_client<MoveToJointPose>(this, "/move_to_joint_pose");
    ibvs_client_ = this->create_client<std_srvs::srv::SetBool>("/ibvs");

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "filtered_target_pose", 10, std::bind(&HomeAndStartIBVS::pose_callback, this, std::placeholders::_1));

    last_pose_time_ = this->now();
    state_ = State::UNINITIALIZED;
    ibvs_enabled_ = false;

    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1e3 / cycle_frequency_hz_)),
                                     std::bind(&HomeAndStartIBVS::run_sequence, this));
  }

private:
  enum class State
  {
    UNINITIALIZED,
    INITIALIZING,
    INITIALIZED,
    HOMING,
    HOMED,
    IBVS
  };

  void run_sequence()
  {
    auto now = this->now();
    auto ms_since_last_pose = (now - last_pose_time_).nanoseconds() / 1e6;

    // Timeout target perso → reset alla fase INITIALIZED
    if (state_ == State::IBVS && ms_since_last_pose > no_target_timeout_ms_)
    {
      disable_ibvs();
      ibvs_enabled_ = false;
      RCLCPP_WARN(this->get_logger(), "Target lost, returning to INITIALIZED.");
      state_ = State::INITIALIZED;
    }

    switch (state_)
    {
      case State::UNINITIALIZED:
        RCLCPP_INFO(this->get_logger(), "Initializing robot (going to initial position).");
        send_goal(initial_position_);
        state_ = State::INITIALIZING;
        break;

      case State::INITIALIZING:
        if (goal_done_)
        {
          goal_done_ = false;
          RCLCPP_INFO(this->get_logger(), "Initialized, going to home position.");
          send_goal(home_position_);
          state_ = State::HOMING;
        }
        break;

      case State::HOMING:
        if (goal_done_)
        {
          goal_done_ = false;
          RCLCPP_INFO(this->get_logger(), "Homing complete, ready for IBVS.");
          state_ = State::HOMED;
        }
        break;

      case State::INITIALIZED:
        if (!goal_active_)  // solo se libero
        {
          RCLCPP_INFO(this->get_logger(), "Re-homing after reset.");
          send_goal(home_position_);
          state_ = State::HOMING;
        }
        break;

      default:
        break;
    }
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr)
  {
    last_pose_time_ = this->now();

    if (state_ == State::HOMED)
    {
      RCLCPP_INFO(this->get_logger(), "Target detected, starting IBVS.");
      enable_ibvs();
      ibvs_enabled_ = true;
      state_ = State::IBVS;
    }
  }

  void send_goal(const std::vector<double>& target_joints)
  {
    if (goal_active_)
    {
      RCLCPP_WARN(this->get_logger(), "Goal already active, skipping new goal.");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      goal_done_ = true;  // evita blocco FSM
      return;
    }

    auto goal_msg = MoveToJointPose::Goal();
    goal_msg.target_joints = target_joints;

    rclcpp_action::Client<MoveToJointPose>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandleMoveToJointPose::SharedPtr goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        goal_done_ = true;
        goal_active_ = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        goal_active_ = true;
      }
    };

    options.result_callback = [this](const GoalHandleMoveToJointPose::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(this->get_logger(), "Reached target joint position");
      else
        RCLCPP_ERROR(this->get_logger(), "Failed to reach target");

      goal_done_ = true;
      goal_active_ = false;
    };

    action_client_->async_send_goal(goal_msg, options);
  }

  void enable_ibvs()
  {
    if (ibvs_client_->wait_for_service(std::chrono::seconds(2)))
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      ibvs_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "IBVS enabled");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "IBVS service not available");
    }
  }

  void disable_ibvs()
  {
    if (ibvs_client_->wait_for_service(std::chrono::seconds(2)))
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      ibvs_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "IBVS disabled");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "IBVS service not available to disable");
    }
  }

  // Members
  rclcpp_action::Client<MoveToJointPose>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ibvs_client_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> initial_position_;

  bool goal_active_ = false;

  std::vector<double> home_position_;
  State state_;
  bool goal_done_ = false;
  bool ibvs_enabled_ = false;
  int cycle_frequency_hz_;
  double no_target_timeout_ms_;
  rclcpp::Time last_pose_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HomeAndStartIBVS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
