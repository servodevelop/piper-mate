# ROS 2 Humble 遥操作

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

**本页导航：**

- [📦 构建工作区](#build)
- [🦾 启动主臂](#leader)
- [▶️ 启动从臂](#follower)
- [🔎 检查话题与源码](#topics)

<a id="build"></a>

## 📦 构建工作区

先在 Ubuntu 22.04 安装 ROS 2 Humble，再完成[连接准备](../docs/README.zh.md#connect)。从仓库根目录执行。仅在 rosdep 尚未初始化时执行 `sudo rosdep init`。

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-venv python3-colcon-common-extensions python3-rosdep
python3 -m venv --system-site-packages .venv-ros
source .venv-ros/bin/activate
python -m pip install -r python-sdk/requirements.txt colcon-common-extensions
rosdep update
rosdep install --from-paths ros2-humble/src --ignore-src -r -y --rosdistro humble
cd ros2-humble
colcon build --symlink-install
source install/setup.bash
```

使用 Ubuntu 22.04 自带的 Python 3.10。上述命令将 colcon 与 SDK 安装在同一环境，构建及两个运行终端都应保持该环境激活。本次重整未进行原生 ROS 运行测试。

<a id="leader"></a>

## 🦾 启动主臂

在仓库根目录打开终端 1：

```bash
source /opt/ros/humble/setup.bash
source .venv-ros/bin/activate
source ros2-humble/install/setup.bash
ros2 run piper_mate driver --ros-args \
  -p port:=/dev/ttyUSB0 -p auto_enable:=false -p gripper_exist:=true
```

`auto_enable:=false` 使主臂保持扭矩关闭状态。驱动初始化时会解锁舵机并清除多圈计数。托住主臂，在启动从臂前检查发布的关节数值。

<a id="follower"></a>

## ▶️ 启动从臂

完成[运动检查](../docs/README.zh.md#operation)后，在仓库根目录打开终端 2：

```bash
source /opt/ros/humble/setup.bash
source .venv-ros/bin/activate
source ros2-humble/install/setup.bash
bash can_activate.sh can0 1000000
ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:=can0 -p auto_enable:=true \
  -p gripper_exist:=true -p gripper_val_mutiple:=2
```

`auto_enable:=true` 可能使从臂开始运动。`gripper_val_mutiple` 是代码中已有的参数拼写，其值用于缩放夹爪指令。请按实际夹爪核对该倍率，不要假定它与 Python 或 LeRobot 路径相同。

<a id="topics"></a>

## 🔎 检查话题与源码

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_ctrl_single
ros2 topic echo /arm_status
```

| 组件 | 源码／接口 |
| --- | --- |
| 主臂可执行程序 `driver` | [piper_mate.py](src/piper_mate/piper_mate/piper_mate.py)；在 `joint_ctrl_single` 发布 `sensor_msgs/JointState` |
| 从臂可执行程序 `piper_single_ctrl` | [piper_ctrl_single_node.py](src/piper/piper/piper_ctrl_single_node.py)；订阅 `joint_ctrl_single` |
| 状态 | `arm_status`、`joint_states_single`、`joint_states_feedback` |

本指南没有选用 `_new.py` 控制器。软件包名保持不变。该工作区提供遥操作节点，不包含完整的 PiPER-Mate URDF、MoveIt 2 配置或 Gazebo 仿真。从臂资料请参阅 [AgileX PiPER ROS](https://github.com/agilexrobotics/piper_ros/tree/humble)。
