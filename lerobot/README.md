# PiPER-Mate and PiPER with LeRobot

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

**On this page:**

- [🧩 Compatibility and calibration](#scope)
- [📦 Install the framework and adapters](#install)
- [🦾 Teleoperate](#teleoperate)
- [📷 Add a camera and record](#record)
- [↩️ Replay a recorded episode](#replay)
- [🧠 Train an ACT policy](#train)
- [🤖 Evaluate your policy](#evaluate)

<a id="scope"></a>

## 🧩 Compatibility and calibration

This workflow uses the [Fashion Star LeRobot fork](https://github.com/servodevelop/lerobot) and the two adapters in this repository. The installation pins framework commit `549afd5f11c19281f9a93466624afd18998f98d2`, whose plugin discovery and CLI entry points were inspected. This is a reproducible source reference, **not a hardware-qualified release**. Python 3.10 on Ubuntu 22.04 is the documentation target; current upstream LeRobot and Jetson compatibility are not established here.

- [Leader package](lerobot_teleoperator_pipermate/): source version `0.0.4`; UART port and baudrate are configurable.
- [Follower package](lerobot_robot_piper/): source version `0.0.1`; its implementation opens `can0` directly even though the configuration declares `can_name`. Use `can0` with this version.
- Both adapters return `True` from `is_calibrated` and have empty `calibrate()` methods. They do **not** guide you through calibration or save calibrated offsets.
- The leader unlocks IDs 0–6 and calls `ResetLoop` during connection. The follower enables itself during connection. Verify mechanical zero and joint directions before starting; review [first-motion guidance](../docs/README.md#operation).

The two adapters are installed from this checkout so the documentation and code refer to the same versions. Do not mix them with unknown versions installed from PyPI.

<a id="install"></a>

## 📦 Install the framework and adapters

Complete [USB and CAN setup](../docs/README.md#connect), install Conda, then run from the piper-mate repository root. The framework clone is a sibling directory:

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

The adapter dependencies include `fashionstar-uart-sdk>=1.3.6`, `python-can` and `piper_sdk`. There is no fully pinned dependency lockfile yet. Save `python -m pip freeze` and the firmware versions with your test results.

<a id="teleoperate"></a>

## 🦾 Teleoperate

The follower may move as soon as the command connects. Begin without a payload and keep the stop control reachable:

```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --teleop.type=lerobot_teleoperator_pipermate \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=pipermate_leader
```

Leader joints are direction-corrected and clamped. The trigger angle is multiplied by 2 and clamped to 0–100 before the follower conversion. This differs from the direct Python workflow; verify your follower gripper range.

<a id="record"></a>

## 📷 Add a camera and record

Find a camera and replace `/dev/video0` with the actual path. Keep its name, resolution, viewpoint and lighting consistent between collection and evaluation:

```bash
lerobot-find-cameras opencv
```

The following example records 10 episodes locally and does not upload a dataset. `local/pipermate-pick` is an example dataset identifier:

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

Use the recording UI to complete or discard episodes. Check the recorded images and joint trajectories before collecting a larger dataset. Ten episodes demonstrate the workflow; they do not imply sufficient training coverage.

<a id="replay"></a>

## ↩️ Replay a recorded episode

Replay commands move the follower without live leader guidance. Clear the workspace and confirm the recorded initial pose before running:

```bash
lerobot-replay \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=piper_follower \
  --dataset.repo_id=local/pipermate-pick \
  --dataset.episode=0
```

<a id="train"></a>

## 🧠 Train an ACT policy

Use a CUDA-capable training environment. If the CUDA check returns `False`, follow the [official PyTorch installation instructions](https://pytorch.org/get-started/locally/) for your platform before training:

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

Resume from a saved checkpoint:

```bash
lerobot-train \
  --config_path=outputs/train/act_pipermate_pick/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

This repository does not distribute a tested PiPER-Mate pretrained policy. Training duration and success rate depend on the dataset, hardware and environment.

<a id="evaluate"></a>

## 🤖 Evaluate your policy

Use the same camera layout and task as training. The following command runs the policy on the physical follower and records evaluation data locally:

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

Validate under supervision, with the stop control ready. See [current validation status](../docs/README.md#validation) and the [LeRobot documentation](https://huggingface.co/docs/lerobot/index) for the framework context.
