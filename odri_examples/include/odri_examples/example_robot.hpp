#pragma once

#include <rclcpp/rclcpp.hpp>

#include "odri_msgs/msg/robot_state.hpp"
#include "odri_msgs/msg/robot_command.hpp"

class ExampleRobot : public rclcpp::Node
{
public:
  explicit ExampleRobot(const std::string &node_name);
  virtual ~ExampleRobot();

private:
  void callbackTimerChangeCommand();
  void callbackTimerPublishCommand();
  void callbackRobotState(const odri_msgs::msg::RobotState::SharedPtr msg);

private:
  rclcpp::TimerBase::SharedPtr timer_change_command_;
  rclcpp::TimerBase::SharedPtr timer_publish_command_;

  rclcpp::Subscription<odri_msgs::msg::RobotState>::SharedPtr sub_robot_state_;
  rclcpp::Publisher<odri_msgs::msg::RobotCommand>::SharedPtr pub_robot_command_;

  std::chrono::high_resolution_clock::time_point t_last_mb_command_;
  std::chrono::milliseconds t_before_zero_commands_;

  odri_msgs::msg::RobotCommand msg_robot_command_;

  struct WaveParams
  {
    double init_pos[2];
    double amplitude;
    double freq;
    double t;
    double dt;
  } wave_params_;

  bool got_initial_position_;
  std::size_t counter_initial_position_;
};
