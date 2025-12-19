# firefly项目文档

![Programming Language](https://img.shields.io/badge/language-Python-blue?style=flat-square) ![Framework](https://img.shields.io/badge/framework-Ros%20Noetic-orange?style=flat-square) ![Hardware Compatibility](https://img.shields.io/badge/hardware-piper%2ffirefly-green?style=flat-square) ![OS Support](https://img.shields.io/badge/OS-Ubuntu%2F22.04-purple?style=flat-square)


[github链接](https://github.com/Welt-liu/lerobot_teleoperator_firefly)

## 一、概述

本文档详细说明如何通过舵机操控臂实现对目标机械臂的远程控制，核心流程包含「机械臂 SDK 下载配置」「三种控制方式」两大部分。操作前请确保硬件连接正常、供电稳定。

## 二、前置准备：下载并配置机械臂 SDK

请下载本项目的仓库，并进入仓库目录。

### 2.1 环境要求

| 类别       | 要求                                                    |
| ---------- | ------------------------------------------------------- |
| 操作系统   | Ubuntu 22.04 (ROS2 HUMBLE)                              |
| 操作方式   | 1. ROS2 HUMBLE<br/>2. Lerobot<br/>3. Python SDK         |
| 机械臂驱动 | [CH340](https://www.wch.cn/downloads/CH341SER_EXE.html) |

### 2.2 检验可用性

```bash
TODO
```


### 三、控制方式

任选其一

[ROS2 HUMBLE](./ROS2_HUMBLE/README.md)

[Lerobot](./Lerobot/README.md)

[Python SDK](./Python_SDK/README.md)
