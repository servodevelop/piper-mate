# lerobot_teleoperator_firefly

[资料一览](./资料一览.md)

## Getting Started

3.运行piper sdk里提供的代码，确认piper已经可以正常工作。

4.安装lerobot， lerobot_teleoperator_firefly 的安装包，teleoperator

```bash
pip install lerobot                         # 安装lerobot
#pip install lerobot_robot_piper             # 安装lerobot_robot_piper
#pip install lerobot_teleoperator_firefly    # 安装lerobot_teleoperator_firefly
cd lerobot_teleoperator_firefly
pip install ./
```

5.  运行遥操作代码。   

```bash
lerobot-teleoperate \
    --robot.type=lerobot_robot_piper \
    --robot.can_name=can0 \
    --robot.id=lerobot_robot_piper \
    --teleop.type=lerobot_teleoperator_firefly \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=lerobot_teleoperator_firefly \
    --teleop.baudrate=1000000
```

