// Copyright 2026 <Robomanipal>

#pragma once

#include <iostream>
#include "motion_sdk/usb_serial.hpp"

namespace motion_sdk
{

class Stepper
{
public:
  explicit Stepper(USBSerial & serial);

  bool ping();
  bool setPosition(float angle_deg);
  bool getPosition(float & angle_deg);

private:
  USBSerial & serial_;
};

}  // namespace motion_sdk
