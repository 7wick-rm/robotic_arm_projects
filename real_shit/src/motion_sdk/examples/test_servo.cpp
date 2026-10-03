// Copyright 2026 <Robomanipal>

#include <chrono>
#include <iostream>
#include <thread>
#include "motion_sdk/servo.hpp"
#include "motion_sdk/usb_serial.hpp"

using motion_sdk::Servo;
using motion_sdk::USBSerial;

int main()
{
  USBSerial serial("/dev/ttyACM0", 115200);

  if (!serial.open())
  {
    std::cerr << "Failed to open serial\n";
    return 1;
  }

  Servo servo(serial, 3);

  if (!servo.ping())
  {
    std::cerr << "PING failed\n";
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  for (int i = 0; i <= 120; i = i + 20)
  {
    float temp = 0.0f;
    servo.getPosition(temp);
    std::cout << "Start Position: " << temp << " deg\n";
    servo.setPosition(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    servo.getPosition(temp);
    std::cout << "End Position: " << temp << " deg\n";

    std::cout << "============================\n";
  }

  for (int i = 120; i >= 0; i = i - 20)
  {
    float temp = 0.0f;
    servo.getPosition(temp);
    std::cout << "Start Position: " << temp << " deg\n";
    servo.setPosition(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    servo.getPosition(temp);
    std::cout << "End Position: " << temp << " deg\n";

    std::cout << "============================\n";
  }

  float pos = 0.0f;
  if (servo.getPosition(pos))
  {
    std::cout << "Position: " << pos << " deg\n";
  }
  else
  {
    std::cerr << "GET_POS failed\n";
  }

  return 0;
}
