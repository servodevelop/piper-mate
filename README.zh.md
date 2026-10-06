<h1 align="center">🦾 PiPER-Mate</h1>

<p align="center"><strong>面向 AgileX PiPER 的 6+1 自由度示教主臂</strong><br>手动示教 · 遥操作 · 机器人学习数据采集</p>

<p align="center">
  <a href="hardware/"><img src="https://img.shields.io/badge/DOF-6%2B1-16A085?style=flat-square" alt="6+1 DoF"></a>
  <a href="python-sdk/"><img src="https://img.shields.io/badge/Python-SDK-3776AB?style=flat-square" alt="Python SDK"></a>
  <a href="ros2-humble/"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square" alt="ROS 2 Humble"></a>
  <a href="lerobot/"><img src="https://img.shields.io/badge/LeRobot-Integration-FFD21E?style=flat-square" alt="LeRobot"></a>
</p>

<p align="center">🌐 <a href="README.md">English</a> / <strong>简体中文</strong></p>

<p align="center"><a href="https://fashionstar.com.hk/robot-arm/piper-mate/">🌐 产品介绍</a> · <a href="https://fashionstar.com.hk/store/product/robot-arm-piper-mate/">🛒 购买</a> · <a href="https://fashionstar.com.hk/support/">💬 技术支持</a></p>

<p align="center"><a href="https://fashionstar.com.hk/store/product/robot-arm-piper-mate/"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp" alt="PiPER-Mate leader arm" width="480"></a></p>

## 📑 目录

- [📖 项目介绍](#introduction)
- [🔧 规格](#specifications)
- [🛒 购买入口](#where-to-buy)
- [🚀 开始使用](docs/README.zh.md)
- [⚙️ 硬件、BOM 与打印文件](hardware/)
- [🐍 Python SDK](python-sdk/)
- [🦾 ROS 2 Humble](ros2-humble/)
- [🤗 LeRobot 数据采集与训练](lerobot/)
- [🛠️ 仓库目录结构](#repository-structure)
- [🔗 相关项目](#related-projects)

<a id="introduction"></a>

## 📖 项目介绍

PiPER-Mate 是 **[Fashion Star](https://fashionstar.com.hk/)** 开发的紧凑型示教主臂，采用 AgileX PiPER 的关节布局，用于手动遥操作和示范数据采集。

- **六关节加手柄输入：** 单手引导机械臂，同时控制从臂夹爪。
- **三条软件路径：** Python 直接控制、ROS 2 Humble 节点，以及用于采集示范和训练策略的 LeRobot 适配器。
- **制作与改装：** 提供装配 BOM、STEP 模型、尺寸图、原生 CAD 源文件和 3MF 打印项目。

本仓库维护 PiPER-Mate 主臂资料及 PiPER 对接代码。从臂需另备硬件、电源和 CAN 接口。首次使用前请阅读[连接步骤与当前验证范围](docs/README.zh.md)；项目整体许可[尚待确认](LICENSE.md)。

<a id="specifications"></a>

## 🔧 规格

| 规格 | PiPER-Mate 主臂 |
| --- | --- |
| 臂展／自由度 | 400 mm／6 个手臂关节 + 1 个手柄输入 |
| 关节舵机 | RP6-U15H-M × 7（装配 BOM） |
| 位置检测 | 12-bit 绝对值编码器 |
| 通信 | UART 总线、UC-01 USB 接口；示例波特率 1,000,000 |
| 主臂电源 | 12 V、3 A、DC5521 |
| 机械臂重量 | 500 g（商品规格） |
| 从臂连接 | AgileX PiPER 经 USB 转 CAN 连接，1 Mbit/s |

产品参数参考[商城规格](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/)。机械运动范围与软件指令限位在[硬件资料](hardware/README.zh.md#specifications)中分别列出。Python 示例设置了 100 Hz 循环目标，该设置不代表实测端到端频率。

<a id="where-to-buy"></a>

## 🛒 购买入口

前往 [PiPER-Mate 商品页](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/)选择 **已装配整机（Fully Assembled）** 或 **DIY 套件（DIY Kit）**。包装内容、电源适配器是否附带、价格与交期以所选配置为准。

自行制作请从[硬件 BOM 与文件](hardware/README.zh.md)开始。

<a id="repository-structure"></a>

## 🛠️ 仓库目录结构

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

## 🔗 相关项目

- [Fashion Star UART/RS-485 总线舵机 SDK](https://github.com/servodevelop/servo-uart-rs485-sdk)
- [Fashion Star CAN 总线舵机 SDK](https://github.com/servodevelop/servo-canbus-sdk)
- [Star Arm 102](https://github.com/servodevelop/Star-Arm-102) — LD／HD 主臂与 FL 从臂，以及配套硬件和机器人学习资料。
- [StarAI Arm](https://github.com/servodevelop/lerobot-starai) — Violin 主臂搭配 Viola／Cello 从臂。
- [AgileX PiPER SDK](https://github.com/agilexrobotics/piper_sdk) · [PiPER ROS 2](https://github.com/agilexrobotics/piper_ros/tree/humble)
- [LeRobot](https://github.com/huggingface/lerobot) · [Fashion Star 集成分支](https://github.com/servodevelop/lerobot)

感谢包括 [Welt-liu](https://github.com/Welt-liu) 在内的原始贡献者，以及 AgileX 与 LeRobot 社区。

[贡献指南](CONTRIBUTING.md) · [反馈问题](https://github.com/servodevelop/piper-mate/issues) · [许可范围](LICENSE.md)
