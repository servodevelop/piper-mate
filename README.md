# PiPER-Mate

[中文](README.zh.md)

## Ecosystem

piper-mate is the main repository for the PiPER-Mate robotic arm project within the Star Arm 102 ecosystem, covering leader-follower teleoperation with Python SDK, ROS 2, and LeRobot workflows.

- 🔗 [Star Arm 102 Series Hub](https://fashionstar.com.hk/robot-arm/star-arm-102/)
- 🐙 [Main Repo: Star-Arm-102](https://github.com/servodevelop/Star-Arm-102)

![Programming Language](https://img.shields.io/badge/language-Python-blue?style=flat-square)
![Framework](https://img.shields.io/badge/framework-ROS2%20Humble-orange?style=flat-square)
![Hardware](https://img.shields.io/badge/hardware-PiPER%20Mate%20%2B%20Piper-green?style=flat-square)
![OS](https://img.shields.io/badge/OS-Ubuntu%2022.04-purple?style=flat-square)
![License](https://img.shields.io/badge/license-MIT-yellow?style=flat-square)

---

## 📖 Project Overview

PiPER-Mate is a robotic arm teleoperation project that uses a **PiPER Mate robotic arm** to remotely control a **Piper robotic arm** in real time. It provides three control methods for robotics research, teleoperation teaching, and AI training data collection.

### ✨ Key Features

- 🤖 **Multiple control methods**: ROS 2 Humble, the LeRobot framework, and Python SDK
- ⚡ **High control frequency**: real-time teleoperation at up to 100 Hz
- 🔄 **Automatic joint mapping**: converts PiPER Mate joint angles into Piper control commands
- 🛡️ **Safety protection**: built-in joint limits, torque control, and exception detection
- 🎯 **Gripper control**: optional coordinated gripper control

<a id="product-specifications"></a>

### Product Specifications

| Specification | PiPER-Mate leader |
| --- | --- |
| Reach | 400 mm |
| DoF | 6 arm joints + 1 trigger |
| Product mechanical travel | Joint 0: −154° to +154°<br>Joint 1: 0° to 195°<br>Joint 2: −175° to 0°<br>Joint 3: −102° to +102°<br>Joint 4: −142° to +142°<br>Joint 5: −120° to +120° |
| Joint servos | RP6-U15H-M × 7 |
| Servo specifications | Voltage range: 9.0–12.6 V<br>Motor type: coreless<br>Encoder: 12-bit absolute encoder<br>Gear ratio: 257:1<br>Housing: all-aluminum alloy<br>Communication protocol: UART bus<br>Static torque: 15 kg-cm<br>Dynamic torque: 6 kg-cm<br>No-load speed: 100 rpm<br>Stall current: 2 A<br>Dimensions: 31.5 × 21 × 27.6 mm<br>Servo weight: 41 g |
| Communication | UART bus; UC-01 USB interface; examples use 1,000,000 baud |
| Power | 12 V, 3 A, DC5521 |
| Arm weight | 500 g |
| Follower connection | AgileX PiPER over USB-to-CAN, 1 Mbit/s |

For joint numbers, servo IDs and software names, see [Joint and servo ID mapping](docs/specifications.md#joint-ids-and-names).

---

## 🚀 Quick Start

### Requirements

| Item | Requirement |
|------|------|
| Operating system | Ubuntu 22.04 |
| ROS version | ROS2 Humble |
| Hardware | PiPER Mate robotic arm + Piper robotic arm |
| Driver | [CH340 USB Driver](https://www.wch.cn/downloads/CH341SER_EXE.html) |

### Installation

#### Method 1: Python SDK (recommended for beginners)

```bash
# 1. Install dependencies
sudo apt update && sudo apt install can-utils ethtool
sudo pip install pyserial fashionstar-uart-sdk piper-sdk python-can scipy

# 2. Configure the CAN interface
cd piper-mate
bash find_all_can_port.sh
bash can_activate.sh can0 1000000

# 3. Run the program
sudo chmod 777 /dev/ttyUSB*
python3 ./Python_SDK/piper_pipermate.py
```

#### Method 2: ROS 2 Humble

```bash
# 1. Install ROS 2 dependencies
cd ROS2_HUMBLE
colcon build
source install/setup.bash

# 2. Start nodes (two terminals required)
# Terminal 1: start the PiPER Mate driver
ros2 run piper_mate driver --ros-args -p port:=/dev/ttyUSB0 -p auto_enable:=false

# Terminal 2: start Piper control
bash can_activate.sh can0 1000000
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true
```

#### Method 3: LeRobot framework

```bash
# See Lerobot/README.md for configuration instructions
```

---

## 📂 Project Structure

```text
PiPER-Mate/
├── Python_SDK/                  # Python SDK control workflow
│   ├── piper_pipermate.py       # Main control program
│   └── README.md                # Detailed usage documentation
├── ROS2_HUMBLE/                 # ROS 2 control workflow
│   ├── src/piper/               # Piper driver node
│   ├── src/piper_mate/          # Piper_mate driver node
│   ├── src/piper_msgs/          # Piper message definitions
│   └── README.md                # ROS 2 usage documentation
├── Lerobot/                     # LeRobot control workflow
│   ├── lerobot_robot_piper/     # Piper robot configuration
│   ├── lerobot_teleoperator_pipermate/  # Teleoperator
│   └── piper-star_en.md         # LeRobot documentation (English)
│   └── piper-star.md            # LeRobot documentation
│   └── README.md                # Usage steps
├── docs/                        # Joint, servo ID and software-name mapping
├── hardware/                    # Hardware resources and BOM
├── can_activate.sh              # CAN interface activation (repository root)
├── can_config.sh                # CAN interface configuration
└── README.md                    # This document
```

---

## 🎯 Control Method Comparison

| Feature | Python SDK | ROS2 HUMBLE | Lerobot |
|------|------------|-------------|---------|
| Difficulty | ⭐ Easy | ⭐⭐⭐ Medium | ⭐⭐⭐⭐⭐ Advanced |
| Real-time performance | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| Extensibility | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| Use cases | Quick testing, teaching | Robotic system integration | AI training, research |

---

## 🔧 Hardware Connection

### Connection Topology

~~~text
PiPER Mate robotic arm <-- USB --> Computer (Ubuntu 22.04)
                                      |
                                     USB
                                      |
Piper robotic arm     <-- CAN --> CAN-to-USB adapter
~~~

---

## ⚠️ Safety Notes

1. **Pre-operation check**: ensure that the arm is clear of obstacles and the workspace is safe.
2. **Emergency stop**: press `Ctrl+C` while the program is running to stop immediately.
3. **Joint limits**: safety angle limits are set automatically to prevent out-of-range motion.
4. **Power management**: ensure stable power to avoid voltage fluctuations.

---

## 🐛 Troubleshooting

### Common Issues

**Q1: Cannot find the `/dev/ttyUSB0` device?**

```bash
# Check USB devices
ls -l /dev/ttyUSB*

# Check the CH340 driver
lsusb | grep CH340

# If the driver is missing, download and install it from the official website.
```

**Q2: Cannot activate the CAN interface?**

```bash
# Find CAN ports
bash find_all_can_port.sh

# Activate the CAN interface manually
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Check CAN interface status
ip link show can0
```

**Q3: Robotic arm connection failed?**

- Check whether the USB cable is loose.
- Confirm that the robotic arm is powered on.
- Check the driver-board switch position (it should point toward the power connector).
- Try a different USB port.

**Q4: Does the program fail to stop when USB disconnects?**

The program includes exception handling. If the PiPER Mate USB connection is lost, it terminates automatically and displays an error message:

```bash
❌ Fatal error: PiPER Mate USB connection lost!
```

---

## 📖 Detailed Documentation

Choose a control method to read its detailed documentation:

- 📘 **[Python SDK Detailed Documentation](./Python_SDK/README.md)** - Recommended: the easiest way to get started
- 📗 **[ROS2 HUMBLE Detailed Documentation](./ROS2_HUMBLE/README.md)** - Suitable for robotic system integration
- 📙 **[Lerobot Detailed Documentation](./Lerobot/README.md)** - Suitable for AI training and research
- ⚙️ **[Hardware Resources](./hardware/README.md)** - STEP, 3MF (coming soon), drawings and BOM

## 📄 License

This project is open source under the [MIT License](LICENSE).

---

## 👥 Authors and Acknowledgements

- **Project maintainer**: [Welt-liu](https://github.com/Welt-liu)
- **Thanks** to the PiPER Mate and Piper teams for hardware support.

---

## 🔗 Related Links

- [PiPER Mate official repository](https://github.com/servodevelop/piper-mate/tree/main)
- [Piper ROS 2 official repository](https://github.com/agilexrobotics/piper_ros/tree/humble/)
- [LeRobot framework](https://github.com/huggingface/lerobot)

---
