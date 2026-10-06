<h1 align="center">🦾 PiPER-Mate</h1>

<p align="center"><strong>6+1 DoF Leader Arm for AgileX PiPER</strong><br>Hand-guided teaching · Teleoperation · Robot-learning data collection</p>

<p align="center">
  <a href="hardware/"><img src="https://img.shields.io/badge/DOF-6%2B1-16A085?style=flat-square" alt="6+1 DoF"></a>
  <a href="python-sdk/"><img src="https://img.shields.io/badge/Python-SDK-3776AB?style=flat-square" alt="Python SDK"></a>
  <a href="ros2-humble/"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square" alt="ROS 2 Humble"></a>
  <a href="lerobot/"><img src="https://img.shields.io/badge/LeRobot-Integration-FFD21E?style=flat-square" alt="LeRobot"></a>
</p>

<p align="center">🌐 <strong>English</strong> / <a href="README.zh.md">简体中文</a></p>

<p align="center"><a href="https://fashionstar.com.hk/robot-arm/piper-mate/">🌐 Product overview</a> · <a href="https://fashionstar.com.hk/store/product/robot-arm-piper-mate/">🛒 Shop</a> · <a href="https://fashionstar.com.hk/support/">💬 Support</a></p>

<p align="center"><a href="https://fashionstar.com.hk/store/product/robot-arm-piper-mate/"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp" alt="PiPER-Mate leader arm" width="480"></a></p>

## 📑 Contents

- [📖 Introduction](#introduction)
- [🔧 Specifications](#specifications)
- [🛒 Where to buy](#where-to-buy)
- [🚀 Get started](docs/README.md)
- [⚙️ Hardware, BOM & print files](hardware/)
- [🐍 Python SDK](python-sdk/)
- [🦾 ROS 2 Humble](ros2-humble/)
- [🤗 LeRobot data collection & training](lerobot/)
- [🛠️ Repository structure](#repository-structure)
- [🔗 Related projects](#related-projects)

<a id="introduction"></a>

## 📖 Introduction

PiPER-Mate is a compact leader arm from **[Fashion Star](https://fashionstar.com.hk/)**, designed around the AgileX PiPER joint layout for hand-guided teleoperation and demonstration collection.

- **Six joints plus a trigger:** guide the arm and control the follower gripper with one hand.
- **Three software paths:** direct Python control, ROS 2 Humble nodes, and LeRobot adapters for recording demonstrations and training policies.
- **Build and modify:** browse the assembly BOM, STEP models, dimension drawings, native CAD source and 3MF print projects.

This repository contains the PiPER-Mate leader resources and PiPER integration code. The follower needs its own hardware, power supply and CAN interface. See [setup and current validation limits](docs/README.md) before first use; the project-wide license is [pending confirmation](LICENSE.md).

<a id="specifications"></a>

## 🔧 Specifications

| Specification | PiPER-Mate leader |
| --- | --- |
| Reach / DoF | 400 mm / 6 arm joints + 1 trigger |
| Joint servos | RP6-U15H-M × 7 (assembly BOM) |
| Position sensing | 12-bit absolute encoder |
| Communication | UART bus, UC-01 USB interface; examples use 1,000,000 baud |
| Leader power | 12 V, 3 A, DC5521 |
| Arm weight | 500 g (product specification) |
| Follower connection | AgileX PiPER via USB-to-CAN, 1 Mbit/s |

Product values refer to the [store specification](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/). Joint ranges and software command limits are listed separately in [hardware resources](hardware/README.md#specifications). The Python example requests a 100 Hz loop; this is a setting, not a measured end-to-end rate.

<a id="where-to-buy"></a>

## 🛒 Where to buy

Choose **Fully Assembled** or **DIY Kit** on the [PiPER-Mate product page](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/). Check the selected option for package contents, adapter availability, price and lead time.

Building your own? Start with the [hardware BOM and files](hardware/README.md).

<a id="repository-structure"></a>

## 🛠️ Repository structure

```text
piper-mate/
├── README.md
├── README.zh.md
├── hardware/
│   ├── step/
│   │   ├── assembly/
│   │   └── parts/
│   ├── printing/3mf/
│   ├── drawings/
│   ├── cad-source/
│   ├── images/
│   └── assembly-guide/
├── python-sdk/
├── ros2-humble/src/
├── lerobot/
│   ├── lerobot_robot_piper/
│   └── lerobot_teleoperator_pipermate/
├── docs/
├── scripts/
├── can_activate.sh
├── can_config.sh
├── can_muti_activate.sh
├── find_all_can_port.sh
├── .github/
├── CONTRIBUTING.md
└── LICENSE.md
```

<a id="related-projects"></a>

## 🔗 Related projects

- [Fashion Star UART/RS-485 Bus Servo SDK](https://github.com/servodevelop/servo-uart-rs485-sdk)
- [Fashion Star CAN Bus Servo SDK](https://github.com/servodevelop/servo-canbus-sdk)
- [Star Arm 102](https://github.com/servodevelop/Star-Arm-102) — LD / HD leaders and FL follower, with hardware and robot-learning resources.
- [StarAI Arm](https://github.com/servodevelop/lerobot-starai) — Violin leader with Viola / Cello followers.
- [AgileX PiPER SDK](https://github.com/agilexrobotics/piper_sdk) · [PiPER ROS 2](https://github.com/agilexrobotics/piper_ros/tree/humble)
- [LeRobot](https://github.com/huggingface/lerobot) · [Fashion Star integration fork](https://github.com/servodevelop/lerobot)

Thanks to the original contributors, including [Welt-liu](https://github.com/Welt-liu), and the AgileX and LeRobot communities.

[Contributing](CONTRIBUTING.md) · [Report an issue](https://github.com/servodevelop/piper-mate/issues) · [License scope](LICENSE.md)
