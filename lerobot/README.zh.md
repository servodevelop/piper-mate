# PiPER-Mate 与 PiPER 的 LeRobot 使用指南

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

**本页导航：**

- [🧩 兼容性与校准](#scope)
- [📦 安装框架与适配器](#install)
- [🦾 遥操作](#teleoperate)
- [📷 添加相机并采集](#record)
- [↩️ 回放采集回合](#replay)
- [🧠 训练 ACT 策略](#train)
- [🤖 评估策略](#evaluate)

<a id="scope"></a>

## 🧩 兼容性与校准

本路径使用 [Fashion Star LeRobot 分支](https://github.com/servodevelop/lerobot)及本仓库中的两个适配器。安装命令固定到框架提交 `549afd5f11c19281f9a93466624afd18998f98d2`，其插件发现机制和命令入口已核对。这是可复现的源码参考，**不是经过实机验证的发布版本**。文档目标环境为 Ubuntu 22.04、Python 3.10；尚未确认与最新上游 LeRobot 或 Jetson 的兼容性。

- [主臂包](lerobot_teleoperator_pipermate/)：源码版本 `0.0.4`，可配置 UART 端口及波特率。
- [从臂包](lerobot_robot_piper/)：源码版本 `0.0.1`；配置虽声明 `can_name`，实现却直接打开 `can0`，本版本请使用 `can0`。
- 两个适配器的 `is_calibrated` 都返回 `True`，`calibrate()` 都为空，**不会**引导校准或保存校准偏移。
- 主臂连接时解锁 ID 0–6 并调用 `ResetLoop`；从臂连接时自动使能。开始前必须核对机械零位与关节方向，并阅读[首次运动说明](../docs/README.zh.md#operation)。

两个适配器从当前工作区安装，保证文档与源码版本对应。不要与来源不明的 PyPI 安装版本混用。

<a id="install"></a>

## 📦 安装框架与适配器

先完成 [USB 与 CAN 设置](../docs/README.zh.md#connect)并安装 Conda，然后在 piper-mate 仓库根目录执行。框架克隆到同级目录：

```bash
conda create -n pipermate-lerobot python=3.10 -y
conda activate pipermate-lerobot
conda install -c conda-forge ffmpeg=7.1.1 -y
git clone https://github.com/servodevelop/lerobot.git ../lerobot-framework
git -C ../lerobot-framework checkout 549afd5f11c19281f9a93466624afd18998f98d2
python -m pip install -e ../lerobot-framework
python -m pip install -e ./lerobot/lerobot_robot_piper -e ./lerobot/lerobot_teleoperator_pipermate
python -m pip check
```

适配器依赖包括 `fashionstar-uart-sdk>=1.3.6`、`python-can` 和 `piper_sdk`。目前没有完全锁定的依赖文件；请随测试结果记录 `python -m pip freeze` 和固件版本。

<a id="teleoperate"></a>

## 🦾 遥操作

命令连接成功后从臂可能立即运动。请先空载运行，并确保能随时操作停止装置：

```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --teleop.type=lerobot_teleoperator_pipermate \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=pipermate_leader
```

主臂关节会进行方向转换及限幅。手柄角度乘以 2 并限制在 0–100 后，再由从臂换算。这与直接 Python 路径不同，请核对实际从臂夹爪行程。

<a id="record"></a>

## 📷 添加相机并采集

查找相机，并用实际路径替换 `/dev/video0`。采集和评估时保持相机名称、分辨率、视角及照明一致：

```bash
lerobot-find-cameras opencv
```

以下示例在本地采集 10 个回合，不上传数据集。`local/pipermate-pick` 为示例数据集标识：

```bash
lerobot-record \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=lerobot_teleoperator_pipermate \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=pipermate_leader \
  --display_data=true \
  --dataset.repo_id=local/pipermate-pick \
  --dataset.single_task="Pick up the cube" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=30 \
  --dataset.push_to_hub=false
```

通过采集界面完成或丢弃回合。扩大数据规模前先检查图像与关节轨迹。10 个回合用于说明流程，不代表训练数据已经充分。

<a id="replay"></a>

## ↩️ 回放采集回合

回放时从臂会按记录运动，不再由主臂实时引导。运行前清理工作区并确认记录的初始姿态：

```bash
lerobot-replay \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --dataset.repo_id=local/pipermate-pick \
  --dataset.episode=0
```

<a id="train"></a>

## 🧠 训练 ACT 策略

使用支持 CUDA 的训练环境。若 CUDA 检查返回 `False`，先按 [PyTorch 官方安装说明](https://pytorch.org/get-started/locally/)为你的平台配置环境：

```bash
python -c "import torch; print(torch.cuda.is_available())"
lerobot-train \
  --dataset.repo_id=local/pipermate-pick \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/act_pipermate_pick \
  --job_name=act_pipermate_pick \
  --wandb.enable=false
```

从已保存的检查点继续训练：

```bash
lerobot-train \
  --config_path=outputs/train/act_pipermate_pick/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

本仓库没有发布经过验证的 PiPER-Mate 预训练策略。训练时间和成功率取决于数据集、硬件与环境。

<a id="evaluate"></a>

## 🤖 评估策略

沿用训练时的相机布局和任务。以下命令在实体从臂上执行策略，并在本地保存评估数据：

```bash
lerobot-record \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=local/eval_pipermate-pick \
  --dataset.single_task="Pick up the cube" \
  --dataset.num_episodes=10 \
  --dataset.push_to_hub=false \
  --policy.path=outputs/train/act_pipermate_pick/checkpoints/last/pretrained_model
```

评估时需要现场监控，并准备停止装置。请查看[当前验证状态](../docs/README.zh.md#validation)，框架背景参阅 [LeRobot 文档](https://huggingface.co/docs/lerobot/index)。
