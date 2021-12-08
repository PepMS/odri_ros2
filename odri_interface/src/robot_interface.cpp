#include "odri_interface/robot_interface.hpp"

namespace odri_interface
{
RobotInterface::RobotInterface(const std::string& node_name) : rclcpp::Node(node_name)
{
    declareParameters();
    odri_robot_ = odri_control_interface::RobotFromYamlFile(params_.robot_yaml_path);

    Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    odri_robot_->Initialize(des_pos);

    pub_robot_state_     = create_publisher<odri_msgs::msg::RobotState>("robot_state", 1);
    subs_motor_commands_ = create_subscription<odri_msgs::msg::RobotCommand>(
        "robot_command", rclcpp::QoS(1),
        std::bind(&RobotInterface::callbackRobotCommand, this, std::placeholders::_1));

    timer_send_commands_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                             std::bind(&RobotInterface::callbackTimerSendCommands, this));

    t_before_zero_commands_ = std::chrono::milliseconds(100);

    positions_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    velocities_ = positions_;

    des_torques_    = positions_;
    des_positions_  = positions_;
    des_velocities_ = positions_;
    des_pos_gains_  = positions_;
    des_vel_gains_  = positions_;
    max_currents_   = positions_;
}

RobotInterface::~RobotInterface() {}

void RobotInterface::declareParameters()
{
    declare_parameter<std::string>("robot_yaml_path", "");
    declare_parameter<std::string>("adapter_name", "enp61s0");
    declare_parameter<int>("n_slaves", 1);

    get_parameter<std::string>("robot_yaml_path", params_.robot_yaml_path);
    get_parameter<std::string>("adapter_name", params_.adapter_name);
    int n_slaves;
    get_parameter<int>("n_slaves", n_slaves);
    params_.n_slaves = n_slaves;
}

void RobotInterface::callbackTimerSendCommands()
{
    odri_robot_->ParseSensorData();

    positions_  = odri_robot_->joints->GetPositions();
    velocities_ = odri_robot_->joints->GetVelocities();

    robot_state_msg_.header.stamp = get_clock()->now();
    robot_state_msg_.motor_states.clear();

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        odri_msgs::msg::MotorState m_state;

        m_state.position = positions_(i);
        m_state.velocity = velocities_(i);

        robot_state_msg_.motor_states.push_back(m_state);
    }

    odri_robot_->joints->SetTorques(des_torques_);  // WARNING: mixing current and torques. Change the msg
    odri_robot_->joints->SetDesiredPositions(des_positions_);
    odri_robot_->joints->SetDesiredVelocities(des_velocities_);
    odri_robot_->joints->SetPositionGains(des_pos_gains_);
    odri_robot_->joints->SetVelocityGains(des_vel_gains_);
    odri_robot_->joints->SetMaximumCurrents(max_currents_(0));  // WARNING: Max current is common for all joints

    pub_robot_state_->publish(robot_state_msg_);
    odri_robot_->SendCommand();
}

void RobotInterface::callbackRobotCommand(const odri_msgs::msg::RobotCommand::SharedPtr msg)
{
    for (std::size_t i = 0; i < msg->motor_commands.size(); ++i) {
        des_torques_(i)    = msg->motor_commands[i].current_ref;
        des_positions_(i)  = msg->motor_commands[i].position_ref;
        des_velocities_(i) = msg->motor_commands[i].velocity_ref;
        des_pos_gains_(i)  = msg->motor_commands[i].kp;
        des_vel_gains_(i)  = msg->motor_commands[i].kd;
        max_currents_(i)   = msg->motor_commands[i].i_sat;
    }
    t_last_mb_command_ = std::chrono::high_resolution_clock::now();
}

}  // namespace odri_interface

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_interface::RobotInterface> master_board_iface =
        std::make_shared<odri_interface::RobotInterface>("RobotInterface");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(master_board_iface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}