#ifndef MOTOR_CONTROL_RPI_COMMS_H
#define MOTOR_CONTROL_RPI_COMMS_H

#include <cstring>
#include <gpiod.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

class RaspberyPiComms
{


public:

  RaspberyPiComms()
  {  }

  void setup();
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return true; }

  void sendMsg();

  private:
      gpiod_chip* chip;
      gpiod_line* left_wheel;
      gpiod_line* right_wheel;
      rclcpp::Logger logger_ = rclcpp::get_logger("RaspberyPiComms");

};

#endif // MOTOR_CONTROL_RPI_COMMS_H