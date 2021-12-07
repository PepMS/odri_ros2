#include "odri_interface/master_board_wrapper.hpp"

namespace odri_interface
{
MasterBoardWrapper::MasterBoardWrapper(const std::string& node_name) : rclcpp::Node(node_name)
{
    declareParameters();

    master_board_if_ = std::make_unique<MasterBoardInterface>("enp61s0");
    master_board_if_->Init();

    for (size_t i = 0; i < params_.n_slaves; ++i) {
        master_board_if_->motor_drivers[i].motor1->SetCurrentReference(0);
        master_board_if_->motor_drivers[i].motor2->SetCurrentReference(0);
        master_board_if_->motor_drivers[i].motor1->Enable();
        master_board_if_->motor_drivers[i].motor2->Enable();
        master_board_if_->motor_drivers[i].EnablePositionRolloverError();
        master_board_if_->motor_drivers[i].SetTimeout(5);
        master_board_if_->motor_drivers[i].Enable();
    }

    while (!master_board_if_->IsTimeout() && !master_board_if_->IsAckMsgReceived()) {
        master_board_if_->SendInit();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    pub_mb_state_        = create_publisher<odri_msgs::msg::MasterBoardState>("master_board_state", 1);
    subs_motor_commands_ = create_subscription<odri_msgs::msg::MotorCommands>(
        "motor_commands", rclcpp::QoS(1),
        std::bind(&MasterBoardWrapper::callbackMotorCommands, this, std::placeholders::_1));

    timer_pub_mb_state_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                            std::bind(&MasterBoardWrapper::callbackTimerPublishMbState, this));

    motor_commands_.reserve(16);

    t_before_zero_commands_ = std::chrono::milliseconds(100);
}

MasterBoardWrapper::~MasterBoardWrapper() {}

void MasterBoardWrapper::declareParameters()
{
    declare_parameter<std::string>("adapter_name", "enp61s0");
    declare_parameter<int>("n_slaves", 1);

    get_parameter<std::string>("adapter_name", params_.adapter_name);
    int n_slaves;
    get_parameter<int>("n_slaves", n_slaves);
    params_.n_slaves = n_slaves;
}

void MasterBoardWrapper::callbackTimerPublishMbState()
{
    master_board_if_->ParseSensorData();
    mb_state_msg_.header.stamp = get_clock()->now();
    mb_state_msg_.drivers.clear();

    for (std::size_t i = 0; i < params_.n_slaves; ++i) {
        odri_msgs::msg::DriverState driver_msg;

        driver_msg.header.stamp = mb_state_msg_.header.stamp;

        driver_msg.motor1.position                = master_board_if_->motor_drivers[i].motor1->position;
        driver_msg.motor1.velocity                = master_board_if_->motor_drivers[i].motor1->velocity;
        driver_msg.motor1.current                 = master_board_if_->motor_drivers[i].motor1->current;
        driver_msg.motor1.is_enabled              = master_board_if_->motor_drivers[i].motor1->is_enabled;
        driver_msg.motor1.has_index_been_detected = master_board_if_->motor_drivers[i].motor1->has_index_been_detected;
        driver_msg.motor1.index_toogle_bit        = master_board_if_->motor_drivers[i].motor1->index_toggle_bit;

        driver_msg.motor2.position                = master_board_if_->motor_drivers[i].motor2->position;
        driver_msg.motor2.velocity                = master_board_if_->motor_drivers[i].motor2->velocity;
        driver_msg.motor2.current                 = master_board_if_->motor_drivers[i].motor2->current;
        driver_msg.motor2.is_enabled              = master_board_if_->motor_drivers[i].motor2->is_enabled;
        driver_msg.motor2.has_index_been_detected = master_board_if_->motor_drivers[i].motor2->has_index_been_detected;
        driver_msg.motor2.index_toogle_bit        = master_board_if_->motor_drivers[i].motor2->index_toggle_bit;

        mb_state_msg_.drivers.push_back(driver_msg);
    }

    pub_mb_state_->publish(mb_state_msg_);

    if (std::chrono::high_resolution_clock::now() - t_last_mb_command_ > t_before_zero_commands_) {
        // Set 0 commands
        for (int i = 0; i < params_.n_slaves * 2; i++) {
            if (i % 2 == 0) {
                if (!master_board_if_->motor_drivers[i].is_connected)
                    continue;  // ignoring the motors of a disconnected slave

                // making sure that the transaction with the corresponding µdriver board succeeded
                if (master_board_if_->motor_drivers[i].error_code == 0xf) {
                    continue;  // user should decide what to do in that case, here we ignore that motor
                }
            }
            if (master_board_if_->motors[i].IsEnabled()) {
                master_board_if_->motors[i].SetCurrentReference(0.0);
                master_board_if_->motors[i].SetKp(0.0);
                master_board_if_->motors[i].SetKd(0.0);
            }
        }
    } else {
        // Send commands
        for (int i = 0; i < params_.n_slaves * 2; i++) {
            if (i % 2 == 0) {
                if (!master_board_if_->motor_drivers[i].is_connected)
                    continue;  // ignoring the motors of a disconnected slave

                // making sure that the transaction with the corresponding µdriver board succeeded
                if (master_board_if_->motor_drivers[i].error_code == 0xf) {
                    continue;  // user should decide what to do in that case, here we ignore that motor
                }
            }
            if (master_board_if_->motors[i].IsEnabled()) {
                master_board_if_->motors[i].SetPositionReference(motor_commands_[i].position_ref);
                master_board_if_->motors[i].SetVelocityReference(motor_commands_[i].velocity_ref);
                master_board_if_->motors[i].SetCurrentReference(motor_commands_[i].current_ref);
                master_board_if_->motors[i].SetKp(motor_commands_[i].kp);
                master_board_if_->motors[i].SetKd(motor_commands_[i].kd);
                master_board_if_->motors[i].SetSaturationCurrent(motor_commands_[i].i_sat);
            }
        }
    }

    master_board_if_->SendCommand();
}

void MasterBoardWrapper::callbackMotorCommands(const odri_msgs::msg::MotorCommands::SharedPtr msg)
{
    motor_commands_    = msg->motor_commands;
    t_last_mb_command_ = std::chrono::high_resolution_clock::now();
}

}  // namespace odri_interface

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_interface::MasterBoardWrapper> master_board_iface =
        std::make_shared<odri_interface::MasterBoardWrapper>("MasterBoardWrapper");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(master_board_iface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}