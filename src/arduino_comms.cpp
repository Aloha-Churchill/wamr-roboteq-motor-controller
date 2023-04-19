#include "diffdrive_arduino/arduino_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <fcntl.h>
#include <pigpio.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>

#define LEFT_WHEEL_GPIO_PIN 12 // GPIO pin used for left wheel PWM signal
#define RIGHT_WHEEL_GPIO_PIN 18 // GPIO pin used for right wheel PWM signal

#define LEFT_IN_1 20 // GPIO pin used for left wheel PWM signal
#define LEFT_IN_2 16 // GPIO pin used for right wheel PWM signal
#define RIGHT_IN_1 14 // GPIO pin used for left wheel PWM signal
#define RIGHT_IN_2 23 // GPIO pin used for right wheel PWM signal

void ArduinoComms::setup()
{  
    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(logger_, "Failed to initialize pigpio library");
        return;
    }

    gpioSetMode(LEFT_IN_1, PI_OUTPUT);
    gpioSetMode(LEFT_IN_2, PI_OUTPUT);
    gpioSetMode(RIGHT_IN_1, PI_OUTPUT);
    gpioSetMode(RIGHT_IN_2, PI_OUTPUT);

    gpioSetMode(LEFT_WHEEL_GPIO_PIN, PI_OUTPUT);
    gpioSetMode(RIGHT_WHEEL_GPIO_PIN, PI_OUTPUT);

    // Set the PWM frequency to 50 Hz
    gpioSetPWMfrequency(LEFT_WHEEL_GPIO_PIN, 50);
    gpioSetPWMfrequency(RIGHT_WHEEL_GPIO_PIN, 50);

}


void ArduinoComms::sendEmptyMsg()
{

}

void ArduinoComms::readEncoderValues(int&, int&)
{

}


void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    int left_motor_signal = -1*val_1;
    int right_motor_signal = -1*val_2;

    // if ((val_1 > 0 && val_2 > 0) || (val_1 < 0 && val_2 < 0)) {
    //     left_motor_signal = -1*val_1;
    //     right_motor_signal = -1*val_2;
    // }
    // else {
    //     left_motor_signal = -3*val_1;
    //     right_motor_signal = -3*val_2;
    // }


    if(left_motor_signal < 0) {
        gpioWrite(LEFT_IN_1, 0);
        gpioWrite(LEFT_IN_2, 1);
    } else {
        gpioWrite(LEFT_IN_1, 1);
        gpioWrite(LEFT_IN_2, 0);
    }

    if(right_motor_signal < 0) {
        gpioWrite(RIGHT_IN_1, 0);
        gpioWrite(RIGHT_IN_2, 1);
    } else {
        gpioWrite(RIGHT_IN_1, 1);
        gpioWrite(RIGHT_IN_2, 0);
    }


    if (left_motor_signal < 0) {
        left_motor_signal = -left_motor_signal;
    }
    if (right_motor_signal < 0) {
        right_motor_signal = -right_motor_signal;
    }
    if (left_motor_signal > 255) {
        left_motor_signal = 255;
    }
    if (right_motor_signal > 255) {
        right_motor_signal = 255;
    }

    gpioPWM(LEFT_WHEEL_GPIO_PIN, left_motor_signal); 
    gpioPWM(RIGHT_WHEEL_GPIO_PIN, right_motor_signal);

    RCLCPP_INFO(logger_, "left val : %d", left_motor_signal);
    RCLCPP_INFO(logger_, "right val: %d", right_motor_signal);

}


void ArduinoComms::setPidValues(float, float, float, float)
{

}

void ArduinoComms::sendMsg()
{

}
// #include "diffdrive_arduino/arduino_comms.h"
// #include <rclcpp/rclcpp.hpp>
// #include <algorithm>
// #include <fcntl.h>

// #define LEFT_WHEEL_GPIO_PIN 23
// #define RIGHT_WHEEL_GPIO_PIN 24

// void ArduinoComms::setup()
// {  
//     chip = gpiod_chip_open("/dev/gpiochip0");
//     left_wheel = gpiod_chip_get_line(chip, LEFT_WHEEL_GPIO_PIN);
//     right_wheel = gpiod_chip_get_line(chip, RIGHT_WHEEL_GPIO_PIN);

//     // Configure left wheel GPIO pin as input
//     gpiod_line_request_input(left_wheel, "left-wheel");

//     // Configure right wheel GPIO pin as output
//     gpiod_line_request_output(right_wheel, "right-wheel", GPIOD_LINE_ACTIVE_STATE_LOW);
// }


// void ArduinoComms::sendEmptyMsg()
// {

// }

// void ArduinoComms::readEncoderValues(int&, int&)
// {

// }

// int map(int x, int in_min, int in_max, int out_min, int out_max)
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// void ArduinoComms::setMotorValues(int val_1, int val_2)
// {
//     // Translate val_1 and val_2 to PWM signal values between 0 and 255
//     int left_wheel_pwm = map(val_1, -20, 20, 0, 255);
//     int right_wheel_pwm = map(val_2, -20, 20, 0, 255);

//     // Set the PWM signal for the right wheel GPIO pin
//     gpiod_line_set_value(right_wheel, 1);

//     // Set the PWM duty cycle for the left wheel GPIO pin
//     float left_wheel_duty_cycle = static_cast<float>(left_wheel_pwm) / 255.0f;
//     gpiod_line_set_value(left_wheel, left_wheel_duty_cycle);

//     // Wait for the appropriate amount of time for the right wheel based on the right_wheel_pwm value
//     gpiod_line_set_value(right_wheel, 0);
//     float right_wheel_duty_cycle = static_cast<float>(right_wheel_pwm) / 255.0f;
//     gpiod_line_set_value(right_wheel, right_wheel_duty_cycle);

//     RCLCPP_INFO(logger_, "Left wheel duty cycle: %f", left_wheel_duty_cycle);
//     RCLCPP_INFO(logger_, "Right wheel duty cycle: %f", right_wheel_duty_cycle);
// }


// void ArduinoComms::setPidValues(float, float, float, float)
// {

// }

// void ArduinoComms::sendMsg()
// {

// }
