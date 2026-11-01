# 关节、舵机 ID 与软件名称对照

[← 文档目录](./)　|　<sub>[English](specifications.md) / **简体中文**</sub>

PiPER-Mate 具有六个手臂关节和一个手柄输入，作为主臂控制 AgileX PiPER 从臂。

**目录：**

- [🔢 关节与软件名称对照](#joint-ids-and-names)
- [📊 从臂指令限位与方向](#follower-command-limits)

<a id="joint-ids-and-names"></a>

## 关节与软件名称对照

**关节编号与舵机通信 ID 使用不同编号方式。** J1–J6 对应 ID 0–5；手柄在装配树中标为 J7，使用 ID 6。[首页产品规格](../README.zh.md#product-specifications)中的关节 0–5 对应 J1–J6。

| 装配关节 | 舵机 ID | Python SDK | LeRobot | ROS 2 |
| --- | ---: | --- | --- | --- |
| J1 | 0 | `joint1` | `Joint_1` | `joint1` |
| J2 | 1 | `joint2` | `Joint_2` | `joint2` |
| J3 | 2 | `joint3` | `Joint_3` | `joint3` |
| J4 | 3 | `joint4` | `Joint_4` | `joint4` |
| J5 | 4 | `joint5` | `Joint_5` | `joint5` |
| J6 | 5 | `joint6` | `Joint_6` | `joint6` |
| J7（手柄） | 6 | `gripper` | `Gripper` | `gripper` |

- 保持现有舵机 ID 0–6。装配标签 `J7-ID6-RP6-U15H-M (Handle)` 表示手柄输入，用于控制从臂夹爪。
- Python SDK 和 ROS 2 在启用 `gripper_exist` 时包含手柄通道。LeRobot 发布的动作键为 `Joint_1.pos`、`Gripper.pos` 等；表中填写对应的电机名称，并保留实际大小写。
- 名称与 ID 一致不代表单位、缩放和校准相同。各软件路径对手柄输入的换算不同，请参阅对应软件指南。

以上名称与 ID 依据：[Python 映射](../Python_SDK/piper_pipermate.py)、[LeRobot 主臂实现](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/starai_pipermate.py)和 [ROS 2 驱动](../ROS2_HUMBLE/src/piper_mate/piper_mate/piper_mate.py)。

<a id="follower-command-limits"></a>

## 从臂指令限位与方向

下表列出 Python SDK 与 LeRobot 主臂配置中六个手臂通道使用的角度限幅与方向符号。弧度值经四舍五入，仅供对照。主臂机械行程见[产品规格](../README.zh.md#product-specifications)。

| 舵机 ID／Python 关节 | Python／LeRobot 角度限幅 | 近似弧度 | 方向 |
| --- | --- | --- | --- |
| 0 / joint1 | −150° 至 +150° | −2.62 至 +2.62 rad | 反向 |
| 1 / joint2 | 0° 至 180° | 0 至 3.14 rad | 正向 |
| 2 / joint3 | −170° 至 0° | −2.97 至 0 rad | 正向 |
| 3 / joint4 | −100° 至 +100° | −1.75 至 +1.75 rad | 反向 |
| 4 / joint5 | −70° 至 +70° | −1.22 至 +1.22 rad | 正向 |
| 5 / joint6 | −120° 至 +120° | −2.09 至 +2.09 rad | 反向 |

依据：[Python 映射](../Python_SDK/piper_pipermate.py)和 [LeRobot 配置](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py)。未核对从臂前，不要将软件限位扩大至主臂机械行程。

BOM、CAD 装配体、图纸和打印资源见[硬件资料](../hardware/README.zh.md)。后续按所选路径进入 [Python SDK](../Python_SDK/README.md)、[LeRobot](../Lerobot/README.md)或 [ROS 2](../ROS2_HUMBLE/README.md) 指南。
