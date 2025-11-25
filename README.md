This repo tracks the evolution of my personal mobile robot platform (Millibot) across multiple versions.

## V1 — Bluetooth Teleop + ROS2 Bridge

- Arduino-based differential drive robot
- HC-05 Bluetooth module
- Custom Arduino firmware that accepts `V <left_pwm> <right_pwm>` commands
- ROS2 node (`bt_car_bridge`) that:
  - Subscribes to `/cmd_vel`
  - Converts linear/angular velocity into left/right wheel PWM
  - Sends commands over Bluetooth to the robot

More versions coming soon (sensors, SLAM, Nav2, manipulation, PCB, etc).
