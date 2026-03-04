# PiPER-Mate

![Programming Language](https://img.shields.io/badge/language-Python-blue?style=flat-square)
![Framework](https://img.shields.io/badge/framework-ROS2%20Humble-orange?style=flat-square)
![Hardware](https://img.shields.io/badge/hardware-PiPER%20Mate%20%2B%20Piper-green?style=flat-square)
![OS](https://img.shields.io/badge/OS-Ubuntu%2022.04-purple?style=flat-square)
![License](https://img.shields.io/badge/license-MIT-yellow?style=flat-square)

---

## 📖 项目简介

PiPER-Mate 是一个机械臂遥操作控制项目，支持通过 **PiPER Mate 机械臂** 实时远程控制 **Piper 机械臂**。项目提供三种控制方式，适用于机器人研究、遥操作教学、AI训练数据采集等多种场景。

### ✨ 核心特性

- 🤖 **多控制方式**：支持 ROS2 HUMBLE、Lerobot 框架、Python SDK 三种控制模式
- ⚡ **高控制频率**：支持最高 100Hz 的实时遥操作
- 🔄 **关节自动映射**：自动将 PiPER Mate 关节角度转换为 Piper 控制指令
- 🛡️ **安全保护**：内置关节角度限制、力矩控制和异常检测机制
- 🎯 **夹爪控制**：支持可选的夹爪协同控制功能

---

## 🚀 快速开始

### 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS版本 | ROS2 Humble |
| 硬件设备 | PiPER Mate机械臂 + Piper 机械臂 |
| 驱动程序 | [CH340 USB驱动](https://www.wch.cn/downloads/CH341SER_EXE.html) |

### 安装步骤

#### 方式一：Python SDK（推荐新手）

```bash
# 1. 安装依赖
sudo apt update && sudo apt install can-utils ethtool
sudo pip install pyserial fashionstar-uart-sdk piper-sdk python-can scipy

# 2. 配置CAN接口
cd piper-mate
bash find_all_can_port.sh
bash can_activate.sh can0 1000000

# 3. 运行程序
sudo chmod 777 /dev/ttyUSB*
python3 ./Python_SDK/piper_pipermate.py
```

#### 方式二：ROS2 HUMBLE

```bash
# 1. 安装ROS2依赖
cd ROS2_HUMBLE
colcon build
source install/setup.bash

# 2. 启动节点（需要两个终端）
# 终端1：启动PiPER Mate驱动
ros2 run piper_mate driver --ros-args -p port:=/dev/ttyUSB0 -p auto_enable:=false

# 终端2：启动Piper控制
bash can_activate.sh can0 1000000
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true
```

#### 方式三：Lerobot 框架

```bash
# 参考Lerobot/README.md配置说明
```

---

## 📂 项目结构

```bash
PiPER-Mate/
├── Python_SDK/                  # Python SDK控制方式
│   ├── piper_pipermate.py       # 主控制程序
│   └── README.md                # 详细使用文档
├── ROS2_HUMBLE/                 # ROS2控制方式
│   ├── src/piper/               # Piper驱动节点
│   ├── src/piper_mate/          # Piper_mate驱动节点
│   ├── src/piper_msgs/          # Piper消息定义
│   └── README.md                # ROS2使用文档
├── Lerobot/                     # Lerobot框架控制方式
│   ├── lerobot_robot_piper/     # Piper机器人配置
│   ├── lerobot_teleoperator_pipermate/  # 遥操作器
│   └── piper-star_en.md         # Lerobot使用文档（英文）
│   └── piper-star.md            # Lerobot使用文档
│   └── README.md                # 使用步骤
├── can_activate.sh              # CAN接口激活（根目录）
├── can_config.sh                # CAN接口配置
└── README.md                    # 本文档
```

---

## 🎯 控制方式对比

| 特性 | Python SDK | ROS2 HUMBLE | Lerobot |
|------|------------|-------------|---------|
| 难度 | ⭐ 简单 | ⭐⭐⭐ 中等 | ⭐⭐⭐⭐⭐ 复杂 |
| 实时性 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| 扩展性 | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| 适用场景 | 快速测试、教学 | 机器人系统集成 | AI训练、研究 |

---

## 🔧 硬件连接

### 连接拓扑

```bash
┌─────────────────┐         USB          ┌─────────────────┐
│                 │◄────────────────────►│                 │
│   PiPER Mate    │                      │      计算机      │
│     机械臂       │                      │ (Ubuntu 22.04)  │
└─────────────────┘                      └────────┬────────┘
                                                  │
                                                 USB
                                                  │
┌─────────────────┐         CAN          ┌────────┴────────┐
│                 │◄────────────────────►│                 │
│      Piper      │                      │  CAN转USB适配器  │
│      机械臂      │                      │                 │
└─────────────────┘                      └─────────────────┘
```

---

## 📊 关节映射

系统自动将 PiPER Mate 的 6 个关节映射到 Piper 机械臂：

| 关节 | PiPER Mate 角度 | Piper 弧度 | 方向 |
|------|------------------|------------|------|
| Joint1 | -150° ~ 150° | -2.62 ~ 2.62 rad | 反向 |
| Joint2 | 0° ~ 180° | 0 ~ 3.14 rad | 正向 |
| Joint3 | -170° ~ 0° | -2.97 ~ 0 rad | 正向 |
| Joint4 | -100° ~ 100° | -1.75 ~ 1.75 rad | 反向 |
| Joint5 | -70° ~ 70° | -1.22 ~ 1.22 rad | 正向 |
| Joint6 | -120° ~ 120° | -2.09 ~ 2.09 rad | 反向 |

---

## ⚠️ 安全注意事项

1. **操作前检查**：确保机械臂周围无障碍物，工作空间安全
2. **急停控制**：程序运行时按 `Ctrl+C` 可立即停止
3. **关节限制**：系统已自动设置安全角度限制，避免越界运动
4. **电源管理**：确保机械臂供电稳定，避免电压波动

---

## 🐛 故障排除

### 常见问题

**Q1: 找不到 `/dev/ttyUSB0` 设备？**

```bash
# 检查USB设备
ls -l /dev/ttyUSB*

# 检查CH340驱动
lsusb | grep CH340

# 如果没有安装驱动，请从官网下载安装
```

**Q2: CAN 接口无法激活？**

```bash
# 查找CAN端口
bash find_all_can_port.sh

# 手动激活CAN接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 检查CAN接口状态
ip link show can0
```

**Q3: 机械臂连接失败？**

- 检查USB线连接是否松动
- 确认机械臂电源已开启
- 检查驱动板开关位置（应拨向电源接口一侧）
- 尝试更换USB端口

**Q4: USB连接断开时程序不终止？**

程序已添加异常处理，当PiPER Mate USB断开时会自动终止并显示错误信息：

```bash
❌ 致命错误：PiPER Mate USB连接断开！
```

---

## 📖 详细文档

选择你需要的控制方式查看详细文档：

- 📘 **[Python SDK 详细文档](./Python_SDK/README.md)** - 推荐！最简单易用
- 📗 **[ROS2 HUMBLE 详细文档](./ROS2_HUMBLE/README.md)** - 适用于机器人系统集成
- 📙 **[Lerobot 详细文档](./Lerobot/README.md)** - 适用于AI训练和研究

## 📄 许可证

本项目基于 [MIT License](LICENSE) 开源。

---

## 👥 作者与致谢

- **项目维护者**：[Welt-liu](https://github.com/Welt-liu)
- **感谢**：PiPER Mate 和 Piper 团队的硬件支持

---

## 🔗 相关链接

- [PiPER Mate 官方仓库](https://github.com/servodevelop/piper-mate/tree/main)
- [Piper ROS2 官方仓库](https://github.com/agilexrobotics/piper_ros/tree/humble/)
- [Lerobot 框架](https://github.com/huggingface/lerobot)

---
