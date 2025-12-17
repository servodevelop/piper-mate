# FashionStar AgileX ROS2 机械臂驱动使用文档

## 概述

本文档介绍如何启动ROS2驱动节点，通过话题发送接收完成FashionStar机械臂控制松灵机械臂piper

## 系统要求

- Ubuntu 22.04
- ROS2 Humble

## 节点说明

### 1. star_piper节点

负责读取FashionStar机械臂的舵机角度信息，并转换为Piper手臂所需的关节状态格式。

### 2. piper_ctrl_single节点

负责控制Piper机械臂，接收关节状态指令并驱动CAN总线上的电机，该节点为piper官方包，详情介绍请看[Piper ROS2-HUMBLE](https://github.com/agilexrobotics/piper_ros/tree/humble/)

## 安装方法

### 1. 安装依赖

```bash
sudo pip install pyserial
sudo pip install fashionstar-uart-sdk
```

```bash
sudo pip install python-can
sudo pip install scipy
sudo pip install piper_sdk
sudo apt update && sudo apt install can-utils ethtool
```

```bash
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

## 启动步骤

编译

```bash
cd ROS2_HUMBLE
colcon build
source install/setup.bash
```

### 终端1：启动star_piper节点

```bash
source install/setup.bash
sudo chmod 777 /dev/ttyUSB*
# 启动star_piper节点 禁用力矩
ros2 run star_piper driver --ros-args -p port:=/dev/ttyUSB0 -p auto_enable:=false
```

**参数说明：**

- `port:=/dev/ttyUSB0`：机械臂连接的串口设备
- `auto_enable:=false`：手动使能力矩控制（设置为true将会使手臂锁定在当前位置）

**预期输出：**

```bash
[INFO] [star_piper_node]: 初始化FashionStar机械臂，端口: /dev/ttyUSB0
[INFO] [star_piper_node]: 机械臂连接成功
[INFO] [star_piper_node]: 机械臂力矩已禁用
[INFO] [star_piper_node]: FashionStar驱动节点初始化完成
```

### 终端2：配置和启动CAN接口

```bash
source install/setup.bash
# 1. 查找所有CAN端口
bash find_all_can_port.sh
# 2. 激活can0接口（波特率1000000）
bash can_activate.sh can0 1000000
# 3. 启动piper控制节点
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true -p gripper_exist:=true -p gripper_val_mutiple:=2
```

**参数说明：**

- `can_port:=can0`：使用的CAN接口名称
- `auto_enable:=true`：自动使能电机驱动
- `gripper_exist:=true`：启用夹爪控制
- `gripper_val_mutiple:=2`：夹爪值倍数（控制灵敏度）

**预期输出：**

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-9.4:1.0
-------------------START-----------------------
Both ethtool and can-utils are installed.
Expected to configure a single CAN module, detected interface can0 with corresponding USB address 3-9.4:1.0.
Interface can0 is already activated with a bitrate of 1000000.
The interface name is already can0.
-------------------OVER------------------------
[INFO] [piper_ctrl_single_node]: can_port is can0
[INFO] [piper_ctrl_single_node]: auto_enable is True
[INFO] [piper_ctrl_single_node]: gripper_exist is True
[INFO] [piper_ctrl_single_node]: gripper_val_mutiple is 2
[INFO] [piper_ctrl_single_node]: --------------------
[INFO] [piper_ctrl_single_node]: Enable status:False
[INFO] [piper_ctrl_single_node]: joint1: -0.005235987755982988
[INFO] [piper_ctrl_single_node]: --------------------
[INFO] [piper_ctrl_single_node]: joint2: 0.0
[INFO] [piper_ctrl_single_node]: joint3: 0.0
[INFO] [piper_ctrl_single_node]: joint4: -0.05235987755982989
[INFO] [piper_ctrl_single_node]: joint5: -0.041887902047863905
[INFO] [piper_ctrl_single_node]: joint6: 1.6074482410867774
[INFO] [piper_ctrl_single_node]: gripper: -0.0002666666666666667
```

## 验证系统运行

启动完成后，可以通过以下命令验证系统状态：

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看关节状态
ros2 topic echo /joint_ctrl_single

# 查看机械臂状态
ros2 topic echo /arm_status
```

## 故障排除

### 常见问题1：CAN接口无法连接

```bash
# 检查CAN接口状态
ip link show can0

# 如果接口未启动，手动激活
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 常见问题2：机械臂连接失败

- 检查USB线连接
- 确认端口号是否正确（可能是/dev/ttyUSB1等）
- 检查机械臂电源是否开启，驱动版开关拨向电源接口一侧

## 安全注意事项

1. **力矩控制**：启动时确保机械臂周围无障碍物
2. **急停准备**：随时准备物理急停按钮
3. **限位检查**：确保机械臂在安全范围内运动
4. **监控运行**：首次运行时密切观察机械臂行为

## 相关文件说明

- `src/star_piper/star_piper/robo_driver.py`：FashionStar机械臂驱动节点
- `src/piper/piper/piper_ctrl_single_node.py`：Piper机械臂控制节点
- `find_all_can_port.sh`：CAN端口查找脚本
- `can_activate.sh`：CAN接口激活脚本

## 技术支持

如遇问题，请检查：

1. 硬件连接是否正常
2. 权限设置是否正确
3. 日志输出中的错误信息
4. 系统依赖是否完整安装
