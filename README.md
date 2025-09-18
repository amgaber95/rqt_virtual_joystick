# RQT Virtual Joystick

Drive ROS 2 robots without hardware: generate virtual axes and buttons in RQT, streaming `sensor_msgs/Joy` and `geometry_msgs/Twist (cmd_vel)` for teleoperation, simulation, and testing.

![Virtual joystick demo](docs/media/RqtVirtualJoystickDemo.gif)


## Features
- **Keyboard & mouse support** – click/drag the stick or use arrow keys (±0.1 increments) and Space to recenter.
- **Joy + Twist publishing** – stream standard `sensor_msgs/Joy` messages and optional `geometry_msgs/Twist` commands to your robot.
- **Tuning controls** – adjust global/X/Y dead zones, exponential response curves, and return-to-center behaviour.
- **Holonomic override** – enable strafing or temporarily toggle it by holding Shift.
- **Reverse steering fix** – when driving backwards, turning remains intuitive (stick right still turns right).
- **Sticky buttons & live feedback** – latch face buttons (A/B/X/Y) and see their state update in real time.
- **Collapsible panels** – hide rarely used controls to save vertical space.
- **Settings persistence** – topics, rates, tuning, and button preferences are restored automatically.


## Installation
### From source
```bash
cd ~/colcon_ws/src
git clone https://github.com/amgaber95/rqt_virtual_joystick.git

cd ~/colcon_ws
colcon build --symlink-install --packages-select rqt_virtual_joystick
source install/setup.bash
```

## Quick Start
- Launch RQT and add the plugin: **Plugins → Robot Tools → Virtual Joystick**.
- Standalone:
  ```bash
  rqt --standalone rqt_virtual_joystick
  # or
  ros2 run rqt_virtual_joystick rqt_virtual_joystick
  ```
- Choose Joy/Twist topics, enable publishing, and start dragging the stick.

## Topics & QoS
- Joy messages publish on the configured topic (default `/joy`).
- Twist messages publish on the configured topic (default `/cmd_vel`).
- QoS for both publishers: Reliable, Keep Last, depth 1.
- Axis mapping:
  - X axis → left/right (`angular.z` when non-holonomic, `linear.y` when holonomic).
  - Y axis → forward/back (`linear.x`).

## Controls & Shortcuts
- **Mouse** – click/drag the stick; release obeys auto-return mode.
- **Keyboard** – arrow keys nudge (±0.05); Space recenters; Shift temporarily toggles holonomic mode.
- **Buttons** – click A/B/X/Y; enable Sticky Buttons to latch.

## License
BSD 3-Clause

## Maintainer
- Abdelrahman Mahmoud – abdulrahman.mahmoud1995@gmail.com
