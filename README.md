# diffdrive_arduino


This node is designed to provide an interface between a `diff_drive_controller` from `ros_control` and an Arduino running firmware from `ros_arduino_bridge`.


ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


need to change laser_frame --> laser for the s2 lidar topic, then it works


