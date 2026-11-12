# PiPER Mate 与 Piper 机械臂集成控制

[← Parent](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

通过 PiPER Mate 主臂的 USB 串口读取角度，经 CAN 控制 Piper 从臂。本目录提供三个并列的控制脚本：单纯遥操作、遥操作＋固定阻尼、遥操作＋夹爪反馈。

**本页导航：**

- [🧭 选择脚本](#overview)
- [📦 安装依赖](#setup)
- [🔌 配置连接](#quick-start)
- [▶️ 运行三个控制脚本](#run)
- [↔️ 共用映射与控制模式](#mapping)
- [🛠️ CAN 工具脚本](#scripts)
- [🔧 故障排除](#troubleshooting)
- [🛑 操作与停止](#safety)
- [📄 许可证](#license)
- [🤝 贡献](#contributing)
- [📬 联系方式](#contact)

<a id="overview"></a>

## 🧭 选择脚本

三个脚本都可独立完成主从遥操作，区别在于主臂阻尼的来源。**每次只选择一个运行，不需要先启动另一个遥控脚本。**

<a id="features"></a>

| 脚本 | 主臂阻尼 | 适用用途 |
| --- | --- | --- |
| [piper_pipermate.py](piper_pipermate.py) | 启动解锁，无负载反馈 | 基础主从位置遥操作 |
| [mate_damping.py](mate_damping.py) | 六关节固定阻尼，不随从臂负载变化 | 遥操作时增加固定手感阻力 |
| [piper_gripper_feedback.py](piper_gripper_feedback.py) | 仅夹爪随负载变化，合拢重、张开轻 | 遥操作并感受夹爪负载变化 |

先完成下面的安装与连接，再到运行部分选择脚本。控制频率是软件目标值，实际频率、跟随精度和阻尼手感需实机验证。

<a id="setup"></a>

## 📦 安装依赖

### 硬件要求

- PiPER Mate 主臂，通过 USB 串口连接。
- Piper 从臂，通过 CAN 总线连接。
- 支持 SocketCAN 的 USB–CAN 适配器。
- Linux 系统，例如 Ubuntu 22.04；具体 SDK／固件组合需自行验证。

### 软件依赖

以下命令均从**仓库根目录**执行，安装 CAN 工具并使用 Python 虚拟环境：

```bash
sudo apt update
sudo apt install ethtool can-utils python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python-sdk/requirements.txt
```

依赖包括 `pyserial`、`fashionstar-uart-sdk`、`piper-sdk`、`python-can`和 `scipy`。依赖表不是实机验证过的锁定版本；报告测试时用 `python -m pip freeze` 记录环境。

<a id="quick-start"></a>

## 🔌 配置连接

三个脚本共用 USB 串口和 CAN 连接。以下示例将 CAN 接口配置为 `can0`、波特率 `1000000`：

```bash
bash find_all_can_port.sh
bash can_activate.sh can0 1000000
```

[USB／CAN 连接与权限设置](../docs/README.zh.md#connect)。按现有 SDK／固件流程先使能从臂，再运行下面任一脚本。脚本不自动完成从臂使能流程。

<a id="run"></a>

## ▶️ 运行三个控制脚本

下列命令均从**仓库根目录**执行。三个入口相互替代，不要同时运行。

<a id="teleoperation"></a>

### 1. `piper_pipermate.py` — 单纯遥操作

**用途**：读取主臂六关节和可选夹爪角度，发送从臂位置目标；不根据从臂负载给主臂施加反馈阻尼。

**运行**：

```bash
python python-sdk/piper_pipermate.py
```

<a id="configure"></a>

**配置**：在脚本 `main()` 中修改以下值，无对应命令行参数：

```python
PIPERMATE_PORT = "/dev/ttyUSB0"
PIPER_CAN_NAME = "can0"
GRIPPER_EXIST = True
UPDATE_RATE = 100.0
```

依次为主臂串口、从臂 CAN 接口、夹爪开关和目标频率 Hz。`UPDATE_RATE` 决定每轮读写后的休眠时间，实际周期还包括读写耗时。

<a id="damping"></a>

### 2. `mate_damping.py` — 遥操作＋固定阻尼

**用途**：[mate_damping.py](mate_damping.py) 在七轴位置遥操作的基础上，给主臂六关节设置**固定阻尼**。它不根据从臂力矩调整阻尼，不属于负载力反馈。

**运行**：

```bash
python python-sdk/mate_damping.py
```

**配置**：端口、CAN、夹爪开关和目标频率在该脚本 `main()` 中修改，变量名与上面四项相同。固定阻尼在初始化的 `set_damping()` 调用处修改，当前设置如下：

| 主臂 ID | 对应部位 | 固定设置 |
| --- | --- | --- |
| 0 | J1 | `set_damping(..., power=0)` |
| 1–2 | J2–J3 | 200 mW |
| 3–5 | J4–J6 | 100 mW |
| 6 | 夹爪 | 不设置夹爪阻尼 |

无需添加 `--feedback-enable`；该脚本没有对应命令行选项。固定值依据源码核对，手感未在本次修改中实机验证。

<a id="feedback"></a>

### 3. `piper_gripper_feedback.py` — 遥操作＋夹爪反馈

**用途**：[piper_gripper_feedback.py](piper_gripper_feedback.py) 将七轴遥操作和**仅夹爪负载反馈**集成在一个文件中。主臂张开时从臂也张开，只有主臂夹爪 ID 6 接收阻尼；不包含六关节反馈、URDF 或全局 HOLD。

**运行**：

```bash
# Teleoperation + actual gripper damping
python python-sdk/piper_gripper_feedback.py --feedback-enable

# Set leader port, follower CAN interface and loop rate
python python-sdk/piper_gripper_feedback.py --port /dev/ttyUSB0 --can can0 --rate 30 --feedback-enable

# Teleoperation + calculation only; no gripper damping output
python python-sdk/piper_gripper_feedback.py
```

**不加 `--feedback-enable` 仍会遥控从臂**，只是不给主臂施加夹爪阻尼。

处理过程：读取 Piper 夹爪 CAN 力矩反馈 → 力矩滤波和死区／持续确认 → 按主臂动作方向计算目标（合拢重、张开轻）→ 功率滤波和升降速率限制 → 写入主臂夹爪阻尼。方向切换保留力矩滤波历史。

**配置**：手感参数集中在文件顶部，修改后需重启：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `DEFAULT_POWER_MW` | 1500 mW | 合拢总阻尼上限 |
| `GRIPPER_BASE_POWER_MW` | 20 mW | 两种方向的基础阻尼 |
| `GRIPPER_DEADZONE_NM` | 0.15 Nm | 基础负载门槛 |
| `GRIPPER_FULL_SCALE_NM` | 0.30 Nm | 超死区后达到满增量所需的额外力矩 |
| `GRIPPER_OPEN_RATIO` | 0.20 | 张开保留的负载阻尼增量比例 |
| `GRIPPER_OPEN_MAX_POWER_MW` | 240 mW | 张开目标上限 |

日志显示 `HEAVY_LOAD`／`LIGHT_LOAD` 等状态、原始／滤波力矩、当前死区、目标及计算输出功率。Nm 是反馈力矩单位，mW 是主臂 SDK 阻尼参数，两者不能等同。手感参数以当前脚本中的值为准。负载估计、通信和实际手感尚未实机标定。

<a id="mapping"></a>

## ↔️ 共用映射与控制模式

### 关节角度转换

三个脚本的位置遥操作共用以下映射，ID 0–5 对应 J1–J6，发送单位为 0.001°。

| 关节 | 角度限制 | 弧度限制（约） | 方向 |
| --- | --- | --- | --- |
| Joint1 | -150° ~ 150° | -2.6179 ~ 2.6179 | 反向 |
| Joint2 | 0° ~ 180° | 0 ~ 3.14 | 正向 |
| Joint3 | -170° ~ 0° | -2.967 ~ 0 | 正向 |
| Joint4 | -100° ~ 100° | -1.745 ~ 1.745 | 反向 |
| Joint5 | -70° ~ 70° | -1.22 ~ 1.22 | 正向 |
| Joint6 | -120° ~ 120° | -2.09439 ~ 2.09439 | 反向 |

### 夹爪控制

ID 6 的角度按 0–45° 对应 0–0.08 m 换算，再转换成夹爪指令所需的微米。该比例换算没有额外显式限幅，运行前核对实际手柄行程。夹爪位置遥控和指令中的固定力矩字段，不代表已实现夹爪负载反馈。

### 控制模式

三个脚本传入 Piper SDK 的参数为 `ctrl_mode=0x01`、`move_mode=0x01`、`move_spd_rate_ctrl=100` 和 `is_mit_mode=0xAD`；具体模式含义以匹配的 SDK／固件为准。

<a id="scripts"></a>

## 🛠️ CAN 工具脚本

这些脚本位于仓库根目录。

| 脚本 | 用途 |
| --- | --- |
| [can_activate.sh](../can_activate.sh) | 单接口激活、比特率设置、重命名和 USB 硬件地址匹配 |
| [can_config.sh](../can_config.sh) | 单／多 CAN 配置、USB 端口映射和比特率设置 |
| [can_muti_activate.sh](../can_muti_activate.sh) | 多设备配置、重复名称检查和交互确认 |
| [find_all_can_port.sh](../find_all_can_port.sh) | 列出 CAN 接口及 USB 端口信息 |

<a id="troubleshooting"></a>

## 🔧 故障排除

### 常见问题

| 问题 | 检查方法 |
| --- | --- |
| CAN 接口未找到 | 检查适配器连接，运行 `bash find_all_can_port.sh`，核对 USB 端口映射 |
| 串口权限不足 | 检查设备属组和 `dialout` 用户组权限，见连接指南 |
| PiPER Mate 连接失败 | 核对 `/dev/ttyUSB0`、设备权限和 Fashion Star SDK 安装 |
| 从臂不跟随 | 核对 CAN 名称、比特率以及从臂的使能／固件状态 |

### 调试信息

原遥操作脚本打印映射后的关节／夹爪目标及错误信息。这些值不是独立测得的从臂实时位置；原脚本没有实际频率统计。串口或 OS 错误可退出循环，其他读取错误可能返回空状态后继续运行。

[连接与故障排除指南](../docs/README.zh.md#connect).

<a id="safety"></a>

## 🛑 操作与停止

1. **操作前检查**：核对主从臂初始姿态、连接和实际运动空间。
2. **停止程序**：`Ctrl+C` 请求退出并尝试关闭连接，不等于硬件急停；脚本不会自动关闭从臂使能或让从臂回零。
3. **主臂阻尼**：单纯遥操作脚本启动时解锁主臂；固定阻尼和夹爪反馈脚本会改变主臂阻力。
4. **关节限制**：软限位只限制六轴目标角度，不保证碰撞、夹伤或断连保护。
5. **只运行一个控制脚本**：这三个脚本共用串口／CAN，不要同时运行。

硬件通信、停止行为、跟随精度和反馈手感尚未完成统一实机验证。

[首次运动、停止与验证状态](../docs/README.zh.md#operation).

<a id="license"></a>

## 📄 许可证

整体软件／硬件许可尚未确定，请以 [LICENSE.md](../LICENSE.md) 记录的组件许可范围为准。旧版 README 的整体 MIT 表述不构成统一许可授权。

<a id="contributing"></a>

## 🤝 贡献

欢迎提交 Issue 和 Pull Request。反馈控制问题时请附 SDK／固件版本、运行命令、参数和日志，并注明是否经过实机测试。

<a id="contact"></a>

## 📬 联系方式

[通过 Issues 联系维护者](https://github.com/servodevelop/piper-mate/issues)。
