# Python SDK 遥操作

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

**本页导航：**

- [📦 安装](#setup)
- [⚙️ 配置](#configure)
- [▶️ 运行](#run)
- [↔️ 映射与限制](#mapping)

<a id="setup"></a>

## 📦 安装

先完成 [USB／CAN 连接](../docs/README.zh.md#connect)。在仓库根目录创建虚拟环境并安装依赖：

```bash
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python-sdk/requirements.txt
```

依赖表依据源码导入和现有 SDK 要求整理，不是经过实机验证的锁定版本。报告测试时请用 `python -m pip freeze`记录已安装版本。

<a id="configure"></a>

## ⚙️ 配置

在 [piper_pipermate.py](piper_pipermate.py) 的 `main()` 中修改现有常量：

```python
PIPERMATE_PORT = "/dev/ttyUSB0"
PIPER_CAN_NAME = "can0"
GRIPPER_EXIST = True
UPDATE_RATE = 100.0
```

这些设置目前没有命令行参数。`UPDATE_RATE` 决定每次读写后的休眠时长，执行读写需要时间，因此实际频率会低于目标。脚本启动时会解锁主臂并清除多圈计数。

<a id="run"></a>

## ▶️ 运行

运行前阅读[首次运动与停止](../docs/README.zh.md#operation)：

```bash
python python-sdk/piper_pipermate.py
```

脚本打开主臂串口与 PiPER CAN 接口并发送关节指令，但没有显式调用 `EnablePiper()`；请按你的 SDK／固件确认从臂使能流程。`Ctrl+C` 退出循环并尝试关闭连接，但不会显式关闭从臂电机使能。

<a id="mapping"></a>

## ↔️ 映射与限制

舵机 ID 0–5 对应 `joint1`–`joint6`；ID 0、3、5 先反向，再应用[关节限幅](../hardware/README.zh.md#specifications)。发送给从臂的单位为 0.001 度。

ID 6 提供手柄输入。Python 将 0–45° 换算为 0–0.08 m，再转换为夹爪指令的微米单位。Python 脚本未对该换算显式限幅，运行前须核对手柄行程。

串口及 OS 异常可退出主循环；其他读取错误返回空状态，循环可能继续。断连行为尚未验证为故障安全机制。详见[验证状态](../docs/README.zh.md#validation)。
