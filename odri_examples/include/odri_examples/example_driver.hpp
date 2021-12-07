#pragma once

#include <rclcpp/rclcpp.hpp>

#include "odri_msgs/msg/master_board_state.hpp"
#include "odri_msgs/msg/motor_commands.hpp"

class ExampleDriver : public rclcpp::Node
{
public:
  explicit ExampleDriver(const std::string &node_name);
  virtual ~ExampleDriver();

private:
  void callbackTimerChangeCommand();
  void callbackTimerPublishCommand();
  void callbackMasterBoardState(const odri_msgs::msg::MasterBoardState::SharedPtr msg);

private:
  rclcpp::TimerBase::SharedPtr timer_change_command_;
  rclcpp::TimerBase::SharedPtr timer_publish_command_;

  rclcpp::Subscription<odri_msgs::msg::MasterBoardState>::SharedPtr sub_mb_state_;
  rclcpp::Publisher<odri_msgs::msg::MotorCommands>::SharedPtr pub_motor_commands_;

  std::chrono::high_resolution_clock::time_point t_last_mb_command_;
  std::chrono::milliseconds t_before_zero_commands_;

  odri_msgs::msg::MotorCommands msg_motor_commands_;

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
