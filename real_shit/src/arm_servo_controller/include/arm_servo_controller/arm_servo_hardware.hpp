// Copyright 2026 <Robomanipal>

#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motion_sdk/servo.hpp"
#include "motion_sdk/usb_serial.hpp"

namespace arm_servo_controller
{

class ArmServoHardware : public hardware_interface::SystemInterface
{
public:
  ~ArmServoHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Binary gripper: the STM firmware does the actual motion ("GRIPPER <id> 0|1") and stops
  // closing by itself when the grasp load passes its threshold. ROS only decides open/close.
  struct GripperCfg
  {
    double open_deg{50.0};     // servo angle when completely open
    double close_deg{114.0};   // servo angle when completely closed
    double open_pos{0.0};      // Jaw joint value (m) when open   (SRDF "open")
    double close_pos{-0.023};  // Jaw joint value (m) when closed (SRDF "close")
    double hysteresis{0.1};    // dead band around the midpoint, as a fraction of the range
    int last_sent{-1};         // -1 = nothing sent yet, 0 = open, 1 = close   (guarded by q_mutex_)
    int cooldown{0};           // control cycles to wait before retrying a failed command (q_mutex_)
    bool busy{false};          // a command is queued or in flight                       (q_mutex_)
  };

  struct GripperRequest
  {
    size_t joint;
    bool close;
  };

  double gripperPosFromDeg(const GripperCfg & g, double deg) const;
  double gripperFraction(const GripperCfg & g, double pos) const;  // 0 = open ... 1 = closed

  // One request/response on the serial bus. The caller must hold bus_mutex_.
  bool sendGripper(size_t i, bool close);

  // Background thread: sends gripper commands so a slow reply (the STM may only answer
  // once the load-limited close has finished) can never stall the control loop.
  void gripperWorker();

  size_t num_joints_;

  std::vector<double> position_state_;
  std::vector<double> velocity_state_;
  std::vector<double> effort_state_;
  std::vector<double> position_command_;

  // Per-joint conversion between the ros2_control unit and servo degrees:
  //   servo_deg = command * deg_per_unit_[i] + deg_offset_[i]
  //   state     = (servo_deg - deg_offset_[i]) / deg_per_unit_[i]
  // Revolute joints (rad): deg_per_unit = 180/pi, offset = 0 (defaults).
  std::vector<double> deg_per_unit_;
  std::vector<double> deg_offset_;

  std::vector<int> is_gripper_;          // 1 for <param name="type">binary_gripper</param>
  std::vector<GripperCfg> gripper_;      // index-aligned with joints

  std::string port_;
  std::vector<int> servo_ids_;

  std::unique_ptr<motion_sdk::USBSerial> serial_;
  std::vector<std::unique_ptr<motion_sdk::Servo>> servos_;

  // The bus is strictly request/response, so only one thread may talk on it at a time.
  // read() and write() use try_lock and simply skip a cycle while a gripper command is in
  // flight (keeping the previous values); the gripper worker takes the lock and waits.
  std::mutex bus_mutex_;

  std::thread gripper_thread_;
  std::mutex q_mutex_;
  std::condition_variable q_cv_;
  std::deque<GripperRequest> queue_;
  bool stop_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("arm_servo_hardware")};
};

}  // namespace arm_servo_controller