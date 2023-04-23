# Motor Controller for WAMR

## File Structure
```
├── CMakeLists.txt
├── controllers
│   └── robot_controller_example.yaml
├── fake_robot_hardware.xml
├── include
│   └── motor_control
│       ├── config.h
│       ├── fake_robot.h
│       ├── motor_control.h
│       ├── rpi_comms.h
│       └── wheel.h
├── launch
│   ├── fake_robot.launch.py
│   └── test_robot.launch.py
├── package.xml
├── README.md
├── robot_hardware.xml
└── src
    ├── diffdrive_robot.cpp
    ├── fake_robot.cpp
    ├── motor_control.cpp
    ├── rpi_comms.cpp
    └── wheel.cpp
```

## Test Chassis
* The test chassis communicates with a motor driver over GPIO pins. Speed is controlled by PWM and direction is controlled by turning an additional two pins either high or low.

## Integrated Robot
* The [RobotEq Controller](https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file) was used for the integrated robot design.
* Serial communication over USB
  * **Future improvement**: Convert from USB to RS232 which offers a more robust platform in the face of electrical disturbances (see pg 180 of the user manuel).

## Launching
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


need to change laser_frame --> laser for the s2 lidar topic, then it works
```

