#include "motor_control/motor_control.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"




DiffDriveRaspberryPi::DiffDriveRaspberryPi()
    : logger_(rclcpp::get_logger("DiffDriveRaspberryPi"))
{}


return_type DiffDriveRaspberryPi::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the rpi
  rpi_.setup();  

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveRaspberryPi::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveRaspberryPi::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


return_type DiffDriveRaspberryPi::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");



  return return_type::OK;
}

return_type DiffDriveRaspberryPi::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveRaspberryPi::read()
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  // Force the wheel position
  l_wheel_.pos = l_wheel_.pos + l_wheel_.vel * deltaSeconds;
  r_wheel_.pos = r_wheel_.pos + r_wheel_.vel * deltaSeconds;

  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveRaspberryPi::write()
{

  if (!rpi_.connected())
  {
    return return_type::ERROR;
  }
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;
  if (deltaSeconds > 1.0 / cfg_.loop_rate)
  {
    RCLCPP_WARN(logger_, "Loop rate too high");
  }

  
  RCLCPP_INFO(logger_, "Right vel = %f", r_wheel_.cmd);
  RCLCPP_INFO(logger_, "Left vel = %f", l_wheel_.cmd);

  l_wheel_.vel = l_wheel_.cmd;
  r_wheel_.vel = r_wheel_.cmd;

  rpi_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  return return_type::OK;

}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveRaspberryPi,
  hardware_interface::SystemInterface
)