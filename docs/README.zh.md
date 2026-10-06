# PiPER-Mate 开始使用

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

**本页导航：**

- [🧰 准备设备](#prepare)
- [🔌 连接 USB 与 CAN](#connect)
- [🚀 选择软件路径](#choose)
- [🛑 首次运动与停止](#operation)
- [🔎 排查问题](#troubleshooting)
- [🧪 验证状态](#validation)

<a id="prepare"></a>

## 🧰 准备设备

准备 Ubuntu 22.04 电脑、PiPER-Mate 主臂、AgileX PiPER 从臂、兼容 Linux SocketCAN 的 USB 转 CAN 适配器，以及各自适用的独立电源。只有 ROS 路径需要 ROS 2 Humble。首次遥操作不需要摄像头或训练用 GPU。

固定两台机械臂并清理运动空间。根据 [AgileX 说明](https://github.com/agilexrobotics/piper_sdk)确认从臂固件与 SDK 的组合。硬件操作流程面向 Linux，尚未验证 macOS 控制。

```text
PiPER-Mate leader -- UC-01 -- USB -------- Ubuntu 22.04 computer
                                              |
AgileX PiPER follower -- CAN -- USB-CAN --------+

Leader: 12 V / 3 A supply
Follower: its own manufacturer-specified supply
```
DIY 套件请先阅读[硬件 BOM](../hardware/README.zh.md#bill-of-materials)和[制作准备](../hardware/assembly-guide/README.zh.md)。

<a id="connect"></a>

## 🔌 连接 USB 与 CAN

首次使用先克隆仓库。以下命令均在仓库根目录执行：

```bash
git clone https://github.com/servodevelop/piper-mate.git
cd piper-mate
```

识别主臂串口，并授予 Linux 用户串口访问权限：

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB*
sudo usermod -aG dialout "$USER"
```

修改用户组后退出登录并重新登录。若存在 `/dev/serial/by-id/...`，优先使用该稳定路径。不要假定每个 `/dev/ttyUSB*` 都是主臂。

为从臂启用对应的 CAN 适配器：

```bash
sudo apt update
sudo apt install can-utils ethtool
bash find_all_can_port.sh
bash can_activate.sh can0 1000000
ip -details link show can0
```

重命名接口前先核对检测到的适配器。多 CAN 设备请查看 [can_config.sh](../can_config.sh) 和 [can_muti_activate.sh](../can_muti_activate.sh)；后者保留原有文件名。这些脚本会修改 Linux 网络接口，可能请求 sudo。

<a id="choose"></a>

## 🚀 选择软件路径

| 使用任务 | 从这里开始 | 当前范围 |
| --- | --- | --- |
| 主臂直接控制从臂 | [Python SDK](../python-sdk/README.zh.md) | 一台主臂和一台 PiPER；在 `main()` 配置端口 |
| ROS 话题集成 | [ROS 2 Humble](../ros2-humble/README.zh.md) | 主臂发布 `joint_ctrl_single`，从臂订阅 |
| 采集示范与 ACT 训练 | [LeRobot](../lerobot/README.zh.md) | 本地适配器和命令示例；实机验证待完成 |

同一时间只运行一条控制路径。启用从臂运动前，先核对连接、零位与运动方向。

<a id="operation"></a>

## 🛑 首次运动与停止

示例启动时可能解锁主臂、清除多圈计数并发送从臂指令。连接前先托住主臂，核对机械零位及舵机 ID 0–6。清除多圈计数不等于完成校准。

首次运行保持空载和足够间隙。当前 Python 和 LeRobot 代码使用了值为 100 的速度控制参数，操作前请核对从臂文档中的具体行为。降低循环频率不等于限制运动速度。

`Ctrl+C` 只请求应用退出，**不能保证从臂电机失能，也不是硬件急停**，通信丢失时尤其如此。请遵循从臂厂商的停止流程，并准备便于操作的实体断电／停止装置。软件关节限幅不具备碰撞检测能力。

<a id="troubleshooting"></a>

## 🔎 排查问题

| 现象 | 检查项 |
| --- | --- |
| 找不到串口 | USB 数据线、UC-01 供电、`lsusb`、`/dev/serial/by-id/` 和内核日志 |
| 权限不足 | 重新登录后的串口属组与 `dialout` 用户组成员关系 |
| CAN 不存在或 BUS-OFF | 正确适配器、1 Mbit/s 波特率、从臂供电、CAN 接线与终端电阻 |
| 方向异常／突然跳变 | 停止运动，核对机械零位、ID 和[映射](../hardware/README.zh.md#specifications) |
| LeRobot 无法加载设备 | 激活目标环境并安装两个本地适配器包 |
| 摄像头失败 | 执行 `lerobot-find-cameras opencv`；核对设备路径、带宽和其他应用占用 |

反馈时请提供仓库提交、系统、依赖版本、从臂固件、连接示意和去除个人路径后的日志。通过 [GitHub Issues](https://github.com/servodevelop/piper-mate/issues)提交。

<a id="validation"></a>

## 🧪 验证状态

| 范围 | 已有依据 | 仍需完成 |
| --- | --- | --- |
| 文档 | 本地链接、双语结构及源码交叉核对 | 面向具体硬件确认操作流程 |
| 控制软件 | Python 源码与 Shell 语法检查 | Ubuntu 22.04／Humble 运行、依赖版本矩阵和实机冒烟测试 |
| 性能 | Python 循环目标为 100 Hz | 持续频率和延迟实测 |
| 硬件资料 | 文件检查；导入文件与源文件字节对比 | CAD 装配重开、跨版本试装、打印及负载验证 |
| LeRobot | 依据文档指定分支快照核对插件注册与命令名 | 校准实现，采集／训练／回放／推理运行 |

当前版本尚未发布实机运行、CAN 时序和 GPU 训练结果。源码结构吻合不代表硬件组合已验证。
