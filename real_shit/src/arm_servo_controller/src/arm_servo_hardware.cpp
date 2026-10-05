// Copyright 2026 <Robomanipal>
#include "arm_servo_controller/arm_servo_hardware.hpp"
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace arm_servo_controller
{

hardware_interface::CallbackReturn ArmServoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  num_joints_ = info_.joints.size();

  position_state_.resize(num_joints_, 0.0);
  velocity_state_.resize(num_joints_, 0.0);
  effort_state_.resize(num_joints_, 0.0);
  position_command_.resize(num_joints_, 0.0);

  port_ = info_.hardware_parameters.at("port");

  // Threshold (in command-interface units, meters for Jaw) above which a
  // commanded position is treated as "open" for mg995 joints. Overridable
  // via a <param name="gripper_open_threshold"> on the <hardware> block;
  // falls back to the 0.05 default set in the header.
  if (info_.hardware_parameters.count("gripper_open_threshold"))
  {
    gripper_open_threshold_ = std::stod(info_.hardware_parameters.at("gripper_open_threshold"));
  }

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);
  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Serial open failed");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    std::string type = info_.joints[i].parameters.count("type")
      ? info_.joints[i].parameters.at("type") : "ax18a";
    joint_type_.push_back(type);

    int id = std::stoi(info_.joints[i].parameters.at("id"));
    servo_ids_.push_back(id);

    if (type == "mg995")
    {
      servos_.push_back(nullptr);  // keep index-aligned, unused for this joint
      gripper_servos_.push_back(
        std::make_unique<motion_sdk::GripperServo>(*serial_, id));
      // no ping(); MG995 can't respond to a position query
    }
    else
    {
      auto servo = std::make_unique<motion_sdk::Servo>(*serial_, id);
      if (!servo->ping())
      {
        RCLCPP_ERROR(logger_, "Servo ping failed id=%d", id);
        return CallbackReturn::ERROR;
      }
      servos_.push_back(std::move(servo));
      gripper_servos_.push_back(nullptr);
    }
  }

  RCLCPP_INFO(logger_, "Arm servo hardware initialized (%ld joints)", num_joints_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmServoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    si.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocity_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_EFFORT, &effort_state_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> ArmServoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    ci.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_command_[i]);
  }
  return ci;
}

hardware_interface::return_type ArmServoHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (joint_type_[i] == "mg995")
    {
      // MG995 has no position/effort/speed feedback at all -- it's a
      // dumb PWM-driven servo switched by the STM firmware. Mirror the
      // last commanded position so allowed_start_tolerance validation
      // and RViz's displayed joint state stay sane; velocity/effort
      // have no meaningful value here, so leave them at zero.
      position_state_[i] = position_command_[i];
      velocity_state_[i] = 0.0;
      effort_state_[i] = 0.0;
      continue;
    }

    float deg{}, eff{}, spd{};
    servos_[i]->getPosition(deg);
    servos_[i]->getEffort(eff);
    servos_[i]->getSpeed(spd);

    position_state_[i] = (deg) * M_PI / 180.0;
    velocity_state_[i] = spd;
    effort_state_[i] = eff;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmServoHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (joint_type_[i] == "mg995")
    {
      bool open = position_command_[i] > gripper_open_threshold_;
      gripper_servos_[i]->setState(open);
      continue;
    }
    servos_[i]->setPosition(position_command_[i] * 180 / M_PI);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_servo_controller

PLUGINLIB_EXPORT_CLASS(arm_servo_controller::ArmServoHardware, hardware_interface::SystemInterface)