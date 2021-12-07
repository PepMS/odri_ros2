#include "odri_examples/example_driver.hpp"

#include <cmath>

ExampleDriver::ExampleDriver(const std::string &node_name) : Node(node_name)
{
  sub_mb_state_ = create_subscription<odri_msgs::msg::MasterBoardState>(
      "master_board_state", rclcpp::QoS(1),
      std::bind(&ExampleDriver::callbackMasterBoardState, this, std::placeholders::_1));

  pub_motor_commands_ = create_publisher<odri_msgs::msg::MotorCommands>("motor_commands", 1);

  timer_change_command_ = create_wall_timer(std::chrono::duration<double>(1),
                                            std::bind(&ExampleDriver::callbackTimerChangeCommand, this));
  timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                             std::bind(&ExampleDriver::callbackTimerPublishCommand, this));

  got_initial_position_ = false;
  counter_initial_position_ = 0;

  wave_params_.amplitude = M_PI;
  wave_params_.freq = 1;
  wave_params_.t = 0;
  wave_params_.dt = 0.001;
}

ExampleDriver::~ExampleDriver() {}

void ExampleDriver::callbackTimerChangeCommand()
{
}

void ExampleDriver::callbackTimerPublishCommand()
{
  msg_motor_commands_.header.stamp = get_clock()->now();
  msg_motor_commands_.motor_commands.clear();

  for (std::size_t i = 0; i < 2; ++i)
  {
    odri_msgs::msg::MotorCommand command;

    command.position_ref = wave_params_.init_pos[i] + wave_params_.amplitude * sin(2 * M_PI * wave_params_.freq * wave_params_.t);
    command.velocity_ref = 2. * M_PI * wave_params_.freq * wave_params_.amplitude * cos(2 * M_PI * wave_params_.freq * wave_params_.t);
    command.kp = 5.;
    command.kd = 0.1;
    command.i_sat = 1.;

    msg_motor_commands_.motor_commands.push_back(command);
  }
  wave_params_.t += wave_params_.dt;

  if (got_initial_position_)
  {
    pub_motor_commands_->publish(msg_motor_commands_);
  }
}

void ExampleDriver::callbackMasterBoardState(const odri_msgs::msg::MasterBoardState::SharedPtr msg)
{
  if (!got_initial_position_)
  {
    wave_params_.init_pos[0] = msg->drivers[0].motor1.position;
    wave_params_.init_pos[1] = msg->drivers[0].motor2.position;

    counter_initial_position_++;
    if (counter_initial_position_ >= 100)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Got initial position. Motor 1: " << wave_params_.init_pos[0] << "Motor 2: " << wave_params_.init_pos[1]);
      got_initial_position_ = true;
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ExampleDriver> example_driver =
      std::make_shared<ExampleDriver>("example_driver");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(example_driver);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}