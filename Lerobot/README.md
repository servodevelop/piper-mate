# lerobot_teleoperator_firefly

[资料一览](./资料一览.md)

## Getting Started

1. [github克隆lerobot](https://github.com/servodevelop/lerobot)

2. 按照piper-star.md文档操作到安装lerobot下第6点安装电机依赖时，请在终端将路径导向该文件夹进行本地安装

    ```bash
    cd ./Lerobot/lerobot_robot_piper
    pip install ./
    cd ./Lerobot/lerobot_teleoperator_pipermate
    pip install ./
    sudo apt update && sudo apt install can-utils ethtool
    ```

3. 继续按照piper-star.md文档操作
