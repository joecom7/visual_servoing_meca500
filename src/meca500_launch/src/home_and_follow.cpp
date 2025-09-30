#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "meca500_interfaces/action/move_to_joint_pose.hpp"
#include <vector>
#include <chrono>

using MoveToJointPose = meca500_interfaces::action::MoveToJointPose;
using GoalHandleMoveToJointPose = rclcpp_action::ClientGoalHandle<MoveToJointPose>;

class HomeAndEnableIBVS : public rclcpp::Node
{
public:
  HomeAndEnableIBVS() : Node("home_and_enable_ibvs")
  {
    this->declare_parameter<int>("cycle_frequency_hz", 1000);
    this->declare_parameter<double>("state_delay_sec", 2.0);  // nuovo parametro
    this->declare_parameter<std::vector<double>>("initial_position", { 1.57, 0.0, 0.0, 0.0, 0.0, 0.0 });
    this->declare_parameter<std::vector<double>>("home_position", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

    initial_position_ = this->get_parameter("initial_position").as_double_array();
    home_position_ = this->get_parameter("home_position").as_double_array();
    cycle_frequency_hz_ = this->get_parameter("cycle_frequency_hz").as_int();
    state_delay_sec_ = this->get_parameter("state_delay_sec").as_double();

    ibvs_client_ = this->create_client<std_srvs::srv::SetBool>("/ibvs");
    action_client_ = rclcpp_action::create_client<MoveToJointPose>(this, "/move_to_joint_pose");
    last_state_change_ = this->now();  // inizializza con la stessa fonte di clock del nodo

    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1e3 / cycle_frequency_hz_)),
                                     std::bind(&HomeAndEnableIBVS::run_sequence, this));
  }

private:
  void run_sequence()
  {
    auto now = this->now();
    if ((now - last_state_change_).seconds() < state_delay_sec_)
    {
      // Se il tempo di delay non è passato, esci
      return;
    }

    switch (step_)
    {
      case 0:  // Move to initial position
        send_goal(initial_position_);
        step_++;
        last_state_change_ = this->now();
        break;

      case 1:  // Move to home position
        if (goal_done_)
        {
          send_goal(home_position_);
          step_++;
          goal_done_ = false;
          last_state_change_ = this->now();
        }
        break;

      case 2:  // Enable IBVS
        if (goal_done_)
        {
          enable_ibvs();
          step_++;
          timer_->cancel();
        }
        break;
    }
  }

  void send_goal(const std::vector<double>& target_joints)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = MoveToJointPose::Goal();
    goal_msg.target_joints = target_joints;

    auto send_goal_options = rclcpp_action::Client<MoveToJointPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [](GoalHandleMoveToJointPose::SharedPtr goal_handle) {
      if (!goal_handle)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected");
      else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by action server");
    };

    send_goal_options.result_callback = [this](const GoalHandleMoveToJointPose::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(this->get_logger(), "Reached target joint position");
      else
        RCLCPP_ERROR(this->get_logger(), "Failed to reach target");

      goal_done_ = true;
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void enable_ibvs()
  {
    if (ibvs_client_->wait_for_service(std::chrono::seconds(2)))
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      ibvs_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Camera following enabled");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "IBVS service not available");
    }
  }

  // Members
  rclcpp_action::Client<MoveToJointPose>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ibvs_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> initial_position_;
  std::vector<double> home_position_;
  int step_ = 0;
  bool goal_done_ = false;
  int cycle_frequency_hz_;
  double state_delay_sec_;
  rclcpp::Time last_state_change_ = rclcpp::Time(0);
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HomeAndEnableIBVS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
