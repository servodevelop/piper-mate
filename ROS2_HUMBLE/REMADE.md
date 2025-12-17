# FashionStar AgileX ROS2 机械臂驱动使用文档

## 概述
本文档介绍如何启动FashionStar机械臂的ROS2驱动节点，包括robo_driver节点和piper控制节点。

## 系统要求
- Ubuntu 22.04
- ROS2 Humble

## 节点说明

### 1. robo_driver节点
负责读取FashionStar机械臂的舵机角度信息，并转换为Piper手臂所需的关节状态格式。

### 2. piper_ctrl_single节点
负责控制Piper机械臂，接收关节状态指令并驱动CAN总线上的电机。

## 安装方法

### 1. 安装依赖

```bash
sudo pip install pyserial
sudo pip install fashionstar-uart-sdk
```

```bash
pip3 install python-can
pip3 install scipy
pip3 install piper_sdk
```

```bash
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```


## 启动步骤

### 步骤1：启动robo_driver节点

```bash
# 启动robo_driver节点（连接到FashionStar机械臂）
ros2 run robo_driver driver --ros-args -p port:=/dev/ttyUSB0 -p auto_enable:=false
```

**参数说明：**
- `port:=/dev/ttyUSB0`：机械臂连接的串口设备
- `auto_enable:=false`：手动使能力矩控制（设置为true可自动使能）

**预期输出：**
```
[INFO] [robo_driver_node]: 初始化FashionStar机械臂，端口: /dev/ttyUSB0
[INFO] [robo_driver_node]: 机械臂连接成功
[INFO] [robo_driver_node]: FashionStar驱动节点初始化完成
```

### 步骤2：配置和启动CAN接口

```bash
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
```
[INFO] [piper_ctrl_single_node]: can_port is can0
[INFO] [piper_ctrl_single_node]: auto_enable is True
[INFO] [piper_ctrl_single_node]: gripper_exist is True
[INFO] [piper_ctrl_single_node]: gripper_val_mutiple is 2
```

## 完整的启动脚本

为了方便使用，可以创建一个启动脚本：

```bash
#!/bin/bash
# start_arm.sh

echo "=== 启动FashionStar机械臂系统 ==="

# 启动robo_driver节点
echo "1. 启动robo_driver节点..."
ros2 run robo_driver driver --ros-args -p port:=/dev/ttyUSB0 -p auto_enable:=false &
ROBO_PID=$!

# 等待robo_driver启动
sleep 2

# 配置CAN接口
echo "2. 配置CAN接口..."
bash find_all_can_port.sh
bash can_activate.sh can0 1000000

# 等待CAN接口配置
sleep 1

# 启动piper节点
echo "3. 启动piper控制节点..."
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=true -p gripper_exist:=true -p gripper_val_mutiple:=2 &
PIPER_PID=$!

echo "=== 系统启动完成 ==="
echo "robo_driver PID: $ROBO_PID"
echo "piper PID: $PIPER_PID"

# 等待用户输入停止
read -p "按回车键停止系统..."

# 停止节点
kill $ROBO_PID
kill $PIPER_PID
echo "系统已停止"
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

### 常见问题2：串口权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或重启生效
```

### 常见问题3：机械臂连接失败
- 检查USB线连接
- 确认端口号是否正确（可能是/dev/ttyUSB1等）
- 检查机械臂电源是否开启

## 安全注意事项

1. **力矩控制**：启动时确保机械臂周围无障碍物
2. **急停准备**：随时准备物理急停按钮
3. **限位检查**：确保机械臂在安全范围内运动
4. **监控运行**：首次运行时密切观察机械臂行为

## 相关文件说明

- `src/robo_driver/robo_driver/robo_driver.py`：FashionStar机械臂驱动节点
- `src/piper/piper/piper_ctrl_single_node.py`：Piper机械臂控制节点
- `find_all_can_port.sh`：CAN端口查找脚本
- `can_activate.sh`：CAN接口激活脚本

## 技术支持

如遇问题，请检查：
1. 硬件连接是否正常
2. 权限设置是否正确
3. 日志输出中的错误信息
4. 系统依赖是否完整安装