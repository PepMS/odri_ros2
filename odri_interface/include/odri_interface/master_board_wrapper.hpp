#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "odri_msgs/msg/driver_state.hpp"
#include "odri_msgs/msg/master_board_state.hpp"
#include "odri_msgs/msg/motor_commands.hpp"

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"

namespace odri_interface
{

class MasterBoardWrapper : public rclcpp::Node
{
    public:
    explicit MasterBoardWrapper(const std::string& node_name);
    virtual ~MasterBoardWrapper();

    private:
    void declareParameters();

    void callbackTimerPublishMbState();
    void callbackMotorCommands(const odri_msgs::msg::MotorCommands::SharedPtr msg);

    private:
    rclcpp::TimerBase::SharedPtr timer_pub_mb_state_;

    rclcpp::Publisher<odri_msgs::msg::MasterBoardState>::SharedPtr pub_mb_state_;
    rclcpp::Subscription<odri_msgs::msg::MotorCommands>::SharedPtr subs_motor_commands_;

    std::unique_ptr<MasterBoardInterface> master_board_if_;

    odri_msgs::msg::MasterBoardState           mb_state_msg_;
    std::vector<odri_msgs::msg::MotorCommand> motor_commands_;

    std::chrono::high_resolution_clock::time_point t_last_mb_command_;
    std::chrono::milliseconds                      t_before_zero_commands_;

    struct Params {
        std::string adapter_name;
        std::size_t n_slaves;
    } params_;
};

}  // namespace odri_interface