# FashionStar与Piper机械臂集成控制

本项目实现了FashionStar机械臂与Piper机械臂的集成控制，通过USB和CAN总线实现实时遥操作。

## 项目概述

该项目允许用户使用FashionStar机械臂作为主控设备，实时控制Piper机械臂的运动。系统通过读取FashionStar机械臂的关节状态，并将其转换为Piper机械臂的控制指令，实现精确的遥操作功能。

## 功能特性

- **实时遥操作**：支持高控制频率
- **关节映射**：自动将FashionStar关节角度转换为Piper关节位置
- **安全保护**：包含关节角度限制和力矩控制
- **夹爪控制**：可选夹爪控制功能

## 项目结构

```bash
fashionstar_agilex/
├── fashionstar_agilex.py     # 主控制程序
├── can_activate.sh           # 单CAN设备激活脚本
├── can_config.sh             # CAN设备配置脚本
├── can_muti_activate.sh      # 多CAN设备激活脚本
└── find_all_can_port.sh      # CAN端口检测脚本
```

## 系统要求

### 硬件要求

- FashionStar机械臂（通过USB连接）
- Piper机械臂（通过CAN总线连接）
- CAN转USB适配器
- Linux系统（推荐Ubuntu 22.04+）

### 软件依赖

```bash
# 安装CAN工具
sudo apt update
sudo apt install ethtool can-utils

# Python依赖
sudo pip install serial fashionstar-uart-sdk piper-sdk python-can scipy
```

## 快速开始

### 1. 配置CAN设备

#### 单CAN设备配置

```bash
# 1. 查找所有CAN端口
bash find_all_can_port.sh
# 2. 激活can0接口（波特率1000000）
bash can_activate.sh can0 1000000
```

### 2. 运行主程序

```bash
python3 ./Python_SDK/fashionstar_agilex.py
```

### 3. 参数配置

在 `fashionstar_agilex.py` 的 `main()` 函数中修改以下参数：

```python
# FashionStar USB端口
FASHIONSTAR_PORT = "/dev/ttyUSB0"

# Piper CAN端口
PIPER_CAN_NAME = "can0"

# 是否包含夹爪
GRIPPER_EXIST = True

# 控制频率（Hz）
UPDATE_RATE = 100.0
```

## 核心功能

### 关节角度转换

系统支持6个关节的角度转换：

| 关节 | 角度限制 | 弧度限制 | 方向 |
|------|----------|----------|------|
| Joint1 | -150° ~ 150° | -2.6179 ~ 2.6179 | 反向 |
| Joint2 | 0° ~ 180° | 0 ~ 3.14 | 正向 |
| Joint3 | -170° ~ 0° | -2.967 ~ 0 | 正向 |
| Joint4 | -100° ~ 100° | -1.745 ~ 1.745 | 反向 |
| Joint5 | -70° ~ 70° | -1.22 ~ 1.22 | 正向 |
| Joint6 | -120° ~ 120° | -2.09439 ~ 2.09439 | 反向 |

### 夹爪控制

夹爪角度转换为米制单位：

- 角度范围：0° ~ 45°
- 对应距离：0m ~ 0.08m

### 控制模式

- **关节控制模式**：0x01
- **速度控制模式**：0x01
- **速度控制比例**：100

## 脚本说明

### can_activate.sh

单CAN设备激活脚本，支持：

- 自动检测CAN接口
- 设置比特率
- 接口重命名
- USB硬件地址匹配

### can_config.sh

通用CAN配置脚本，支持：

- 单CAN和多CAN设备配置
- 灵活的USB端口映射
- 比特率自定义

### can_muti_activate.sh

多CAN设备激活脚本，支持：

- 多设备并行配置
- 重复名称检测
- 交互式确认

### find_all_can_port.sh

CAN端口检测脚本，用于：

- 列出所有CAN接口
- 显示USB端口信息
- 系统兼容性检查

## 故障排除

### 常见问题

1. **CAN接口未找到**
   - 检查CAN设备是否连接
   - 运行 `find_all_can_port.sh` 检测接口
   - 确认USB端口映射正确

2. **权限不足**
   - 使用 `sudo chmod 777 /dev/ttyUSB*` 后运行脚本

3. **FashionStar连接失败**
   - 检查USB端口号
   - 确认FashionStar SDK安装正确
   - 检查设备权限

### 调试信息

程序运行时显示：

- 关节状态实时监控
- 控制频率统计
- 错误信息详细输出

## 安全注意事项

1. **操作前检查**：确保机械臂周围无障碍物
2. **紧急停止**：按Ctrl+C可立即停止程序
3. **力矩控制**：默认禁用FashionStar力矩
4. **关节限制**：系统自动限制关节角度在安全范围内

## 许可证

本项目基于MIT许可证开源。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 联系方式

如有问题请联系项目维护者。
