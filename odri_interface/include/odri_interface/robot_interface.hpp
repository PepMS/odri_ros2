#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "odri_msgs/msg/driver_state.hpp"
#include "odri_msgs/msg/master_board_state.hpp"
#include "odri_msgs/msg/robot_command.hpp"
#include "odri_msgs/msg/robot_state.hpp"

#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"

namespace odri_interface
{

class RobotInterface : public rclcpp::Node
{
    public:
    explicit RobotInterface(const std::string& node_name);
    virtual ~RobotInterface();

    private:
    void declareParameters();

    void callbackTimerSendCommands();
    void callbackRobotCommand(const odri_msgs::msg::RobotCommand::SharedPtr msg);

    private:
    rclcpp::TimerBase::SharedPtr timer_send_commands_;

    rclcpp::Publisher<odri_msgs::msg::RobotState>::SharedPtr      pub_robot_state_;
    rclcpp::Subscription<odri_msgs::msg::RobotCommand>::SharedPtr subs_motor_commands_;

    std::unique_ptr<MasterBoardInterface>          master_board_if_;
    std::shared_ptr<odri_control_interface::Robot> odri_robot_;

    odri_msgs::msg::RobotState                robot_state_msg_;

    std::chrono::high_resolution_clock::time_point t_last_mb_command_;
    std::chrono::milliseconds                      t_before_zero_commands_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;

    Eigen::VectorXd des_torques_;
    Eigen::VectorXd des_positions_;
    Eigen::VectorXd des_velocities_;
    Eigen::VectorXd des_pos_gains_;
    Eigen::VectorXd des_vel_gains_;
    Eigen::VectorXd max_currents_;

    struct Params {
        std::string adapter_name;  // rm (yaml)
        std::size_t n_slaves;      // rm (yaml)
        std::string robot_yaml_path;
    } params_;
};

}  // namespace odri_interface