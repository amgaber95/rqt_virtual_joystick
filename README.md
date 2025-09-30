# 🎮 RQT Virtual Joystick

[![ROS 2](https://img.shields.io/badge/ROS-2%20Humble+-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)](https://www.python.org/)

> A lightweight ROS 2 plugin for smooth, intuitive robot teleoperation — tested on Humble and newer.

**RQT Virtual Joystick** provides an intuitive, low-latency way to drive robots directly from your desktop.
With mouse or keyboard input, it enables smooth, continuous control with configurable sensitivity and support for both holonomic and non-holonomic robots.
Designed for teleoperation, simulation, and rapid prototyping, it replaces rigid step-based control with a natural interface — making robot interaction faster, easier, and more precise.

![Virtual Joystick Demo](docs/media/RqtVirtualJoystickDemo2.gif)

---

## ✨ Features

### 🎯 Core Functionality

* **🖱️ Mouse & Keyboard Control** – Intuitive click-and-drag joystick or precise arrow key nudges (±0.05)
* **📡 Dual Message Publishing** – Simultaneous `sensor_msgs/Joy` and `geometry_msgs/Twist` output
* **🎛️ Virtual Controller Buttons** – A, B, X, Y buttons with sticky latch mode
* **📊 Tabbed Interface** – Separate Joy and Twist workspaces for organized control

### ⚙️ Advanced Configuration

* **🎚️ Fine-tuned Dead Zones** – Global, X-axis, and Y-axis independent dead zone control
* **📈 Exponential Response Curves** – Separate expo curves for precise low-speed control
* **🔄 Return-to-Center Modes** – Both axes, X-only, Y-only, or disabled
* **🚁 Holonomic Drive Support** – Strafe mode with Shift key toggle for omnidirectional robots

### 🎨 User Experience

* **💾 Settings Persistence** – All configurations auto-save and restore
* **📦 Collapsible Panels** – Maximize screen space by hiding unused controls
* **🔄 Live Feedback** – Real-time joystick position and button state display

### 🏗️ Technical Features

* **⚡ Configurable Publish Rates** – Independent rates for Joy and Twist messages (1–100 Hz)
* **🔌 Custom Topic Names** – Flexible topic configuration for any robot setup
* **📝 Message Stamping** – Optional stamped Twist messages 

---

## 📦 Installation

### From Source

> ✅ Tested on **ROS 2 Humble** and newer.

```bash
# Navigate to your ROS 2 workspace
cd ~/colcon_ws/src

# Clone the repository
git clone https://github.com/amgaber95/rqt_virtual_joystick.git

# Install dependencies (if not already available)
rosdep install --from-paths . --ignore-src -r -y

# Build the package
cd ~/colcon_ws
colcon build --symlink-install --packages-select rqt_virtual_joystick

# Source the workspace
source install/setup.bash
```

---

## 🚀 Quick Start

### Launch Options

#### 1. Within RQT

```bash
rqt
# Then: Plugins → Robot Tools → Virtual Joystick
```

#### 2. Standalone Mode

```bash
# Using rqt
rqt --standalone rqt_virtual_joystick

# Or directly
ros2 run rqt_virtual_joystick rqt_virtual_joystick
```

---

## 📡 Topics & Message Types

### Joy Messages (`sensor_msgs/Joy`)

* **Default Topic**: `/joy`
* **Message Fields**:

  * `axes[0]` = X-axis (left/right: -1.0 to 1.0)
  * `axes[1]` = Y-axis (forward/back: -1.0 to 1.0)
  * `buttons[0–3]` = A, B, X, Y button states (0 or 1)
* **QoS Profile**: Reliable, Keep Last (depth: 10)

### Twist Messages (`geometry_msgs/Twist` or `TwistStamped`)

* **Default Topic**: `/cmd_vel`
* **Non-Holonomic Mode**:

  * `linear.x` = Forward/backward velocity (Y-axis)
  * `angular.z` = Turning rate (X-axis)
* **Holonomic Mode**:

  * `linear.x` = Forward/backward velocity (Y-axis)
  * `linear.y` = Left/right strafe velocity (X-axis)
* **Stamped Option**: Adds header with timestamp and configurable frame_id
* **QoS Profile**: Reliable, Keep Last (depth: 10)
* **Scaling Factors**:

  * Linear Scale: multiplier for linear velocities (default: 1.0)
  * Angular Scale: multiplier for angular velocities (default: 1.0)

---

## ⌨️ Controls & Shortcuts

### Mouse Controls

* **Click & Drag** – Move joystick to desired position
* **Release** – Auto-return based on selected mode
* **Button Click** – Toggle A/B/X/Y buttons

### Keyboard Shortcuts

* **↑ ↓ ← →** – Nudge joystick ±0.05 in any direction
* **Space** – Re-center joystick to neutral
* **Shift (Hold)** – Temporarily toggle holonomic mode

### Button Controls
- **Click** – Toggle button state (on/off)
- **Sticky Mode** – Buttons stay pressed until clicked again (like physical latch)
- **Normal Mode** – Buttons release when mouse button is released

## 👨‍💻 Maintainer

**Abdelrahman Mahmoud**

📧 Email: abdulrahman.mahmoud1995@gmail.com  
🐙 GitHub: [@amgaber95](https://github.com/amgaber95)  

---

<div align="center">

### 🌟 If this project helps you, please consider giving it a star! 🌟

[![GitHub stars](https://img.shields.io/github/stars/amgaber95/rqt_virtual_joystick?style=social)](https://github.com/amgaber95/rqt_virtual_joystick/stargazers)

**Made with ❤️ for the ROS 2 Community**

*"Enabling robot control without hardware barriers"*

---

</div>
