# How to use the LeRobot-starai robotic arm in Lerobot

https://github.com/user-attachments/assets/dcbd27da-9e24-4562-b682-ff3212f6ac4a

## Products Introduction

1. **Open-Source & Developer-Friendly**
   It is an open-source, developer-friendly 6+1 DoF robotic arm solution from [Fishion Star Technology Limited](https://fashionrobo.com/).
2. **Integration with LeRobot**
   Designed for integration with [LeRobot Platform](https://github.com/huggingface/lerobot) , which provides PyTorch models, datasets, and tools for imitation learning in real-world robotic tasks — including data collection, simulation, training, and deployment.
3. **Comprehensive Learning Resources**
   Provides comprehensive open-source learning resources like assembly and calibration guides, and example custom grasping tasks to assist users in quickly getting started and developing robotic applications.
4. **Compatible with Nvidia**
   Supports deployment on the reComputer Mini J4012 Orin NX 16GB platform.

## Main Features

- Ready to Go — No Assembly Required. Just Unbox and Dive into the World of AI.

## Initial Environment Setup

For Ubuntu X86:

- Ubuntu 22.04
- CUDA 12+
- Python 3.10
- Troch 2.6

## Installation

### Install LeRobot

1. Environments such as pytorch and torchvision need to be installed based on your CUDA.

    Install Miniconda: For Jetson:

    ```bash
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
    chmod +x Miniconda3-latest-Linux-aarch64.sh
    ./Miniconda3-latest-Linux-aarch64.sh
    source ~/.bashrc
    ```

    Or, For X86 Ubuntu 22.04:

    ```bash
    mkdir -p ~/miniconda3
    cd miniconda3
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
    rm ~/miniconda3/miniconda.sh
    source ~/miniconda3/bin/activate
    conda init --all
    ```

2. Create and activate a fresh conda environment for lerobot

    ```bash
    conda create -y -n lerobot python=3.10 && conda activate lerobot
    ```

3. When using miniconda, install ffmpeg in your environment:

    ```bash
    conda install ffmpeg -c conda-forge
    ```

    This usually installs ffmpeg 7.X for your platform compiled with the libsvtav1 encoder. If libsvtav1 is not supported (check supported encoders with ffmpeg -encoders), you can:

    - [On any platform] Explicitly install ffmpeg 7.X using:

    ```bash
    conda install ffmpeg=7.1.1 -c conda-forge
    ```

4. Install LeRobot:

    ```bash
    cd ~/lerobot && pip install -e .
    ```

    For Jetson Jetpack devices (please ensure that Pytorch-gpu and Torchvision have been installed following Step 5 of [this tutorial](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch) before proceeding with this step):

    ```bash
    conda install -y -c conda-forge "opencv>=4.10.0.84"  # Install OpenCV and other dependencies via conda, only for Jetson Jetpack 6.0+
    conda remove opencv   # Uninstall OpenCV
    pip3 install opencv-python==4.10.0.84  # Install the specified version of OpenCV using pip3
    conda install -y -c conda-forge ffmpeg
    conda uninstall numpy
    pip3 install numpy==1.26.0  # This version needs to be compatible with torchvision
    ```

5. Install Motor Dependencies:

     ```bash
    pip install lerobot_teleoperator_pipermate    # Install pipermate via pip
    pip install lerobot_robot_piper    # Install piper via pip
    sudo apt update && sudo apt install can-utils ethtool
    ```

6. Check Pytorch and Torchvision

    Since installing the lerobot environment via pip will uninstall the original Pytorch and Torchvision and install the CPU versions of Pytorch and Torchvision, you need to perform a check in Python.

    ```python
    import torch
    print(torch.cuda.is_available())
    ```

    If the result is False, you need to reinstall Pytorch and Torchvision according to the [official website tutorial](https://fashionrobo.com/).

### Unboxing

```bash
┌─────────────────┐         USB          ┌─────────────────┐
│                 │◄────────────────────►│                 │
│   PiPER Mate    │                      │      计算机      │
│     机械臂       │                      │ (Ubuntu 22.04)  │
└─────────────────┘                      └────────┬────────┘
                                                  │
                                                 USB
                                                  │
┌─────────────────┐         CAN          ┌────────┴────────┐
│                 │◄────────────────────►│                 │
│      Piper      │                      │  CAN转USB适配器  │
│      机械臂      │                      │                 │
└─────────────────┘                      └─────────────────┘
```

### Configure arm port

Run the following command in the terminal to find USB ports associated to your arms：

```bash
lerobot-find-port
```

For example：

1. Example output when identifying the leader arm's port (e.g., `/dev/tty.usbmodem575E0031751` on Mac, or possibly `/dev/ttyUSB0` on Linux):

> [!NOTE]
>
> If the ttyUSB0 serial port cannot be identified, try the following solutions:
> Removing brltty will resolve the issue.
>
> ```sh
> sudo apt remove brltty
> ```
>
> Finally，use chmod command.
>
> ```sh
> sudo chmod 777 /dev/ttyUSB*
> ```
>

### Configure CAN Device

> [!TIP]
>
> Connect the follower to can0

```bash
# 1. Find all CAN ports
bash find_all_can_port.sh
# 2. Activate can0 interface (baud rate 1000000)
bash can_activate.sh can0 1000000
```

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
```

## Teleoperate

Then you are ready to teleoperate your robot (It won't display the cameras)! Run this simple script :

```bash
lerobot-teleoperate \
    --robot.type=lerobot_robot_piper \
    --robot.can_name=can0 \
    --robot.id=my_awesome_piper_arm \
    --teleop.type=lerobot_teleoperator_pipermate \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=my_awesome_pipermate_arm
```

After the program starts, the Hover Lock Technology remains functional.

## Add cameras

After inserting your two USB cameras, run the following script to check the port numbers of the cameras. It is important to remember that the camera must not be connected to a USB Hub; instead, it should be plugged directly into the device. The slower speed of a USB Hub may result in the inability to read image data.

```bash
lerobot-find-cameras opencv # or realsense for Intel Realsense cameras
```

The terminal will print out the following information. For example, the laptop camera is `index 2`, and the USB camera is `index 4`.

```markdown
--- Detected Cameras ---
Camera #0:
  Name: OpenCV Camera @ /dev/video2
  Type: OpenCV
  Id: /dev/video2
  Backend api: V4L2
  Default stream profile:
    Format: 0.0
    Width: 640
    Height: 480
    Fps: 30.0
--------------------
Camera #1:
  Name: OpenCV Camera @ /dev/video4
  Type: OpenCV
  Id: /dev/video4
  Backend api: V4L2
  Default stream profile:
    Format: 0.0
    Width: 640
    Height: 360
    Fps: 30.0
--------------------

Finalizing image saving...
Image capture finished. Images saved to outputs/captured_images
```

After confirming that the external camera is connected, replace the camera information below with the actual camera details. This will allow you to display the camera feed on your computer during teleoperation:

```bash
lerobot-teleoperate \
    --robot.type=lerobot_robot_piper \
    --robot.can_name=can0 \
    --robot.id=my_awesome_piper_arm \
    --robot.cameras="{ front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}" \
    --teleop.type=lerobot_teleoperator_pipermate \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=my_awesome_pipermate_arm \
    --display_data=true
```

## Record the dataset

> [!TIP]
>
> Use the features of Hugging Face Hub to upload your dataset, and please refer to the following official article link.
>
> [lmitation Learning for Robots](https://huggingface.co/docs/lerobot/il_robots?teleoperate_koch_camera=Command)
>
>

Once you're familiar with teleoperation, you can record your first dataset.

Record 10 episodes.

```bash
lerobot-record \
    --robot.type=lerobot_robot_piper \
    --robot.can_name=can0 \
    --robot.id=my_awesome_piper_arm \
    --robot.cameras="{ up: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30},front: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 30}}" \
    --teleop.type=lerobot_teleoperator_pipermate \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=my_awesome_pipermate_arm \
    --display_data=true \
    --dataset.repo_id=starai/record-test \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=30 \
    --dataset.num_episodes=10 \
    --dataset.push_to_hub=False \
    --dataset.single_task="Grab the black cube"
```

## Replay an episode

```bash
lerobot-replay \
    --robot.type=lerobot_robot_piper \
    --robot.can_name=can0 \
    --robot.id=my_awesome_piper_arm \
    --dataset.repo_id=starai/record-test \
    --dataset.episode=1 # choose the episode you want to replay
```

## Train

Train a policy to control your robot

```bash
lerobot-train \
  --dataset.repo_id=starai/record-test \
  --policy.type=act \
  --output_dir=outputs/train/act_piper_test \
  --job_name=act_piper_test \
  --policy.device=cuda \
  --wandb.enable=False \
  --policy.repo_id=starai/my_policy
```

To resume training from a checkpoint

```bash
lerobot-train \
  --config_path=outputs/train/act_piper_test/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

## Evaluate your policy

Run the following command to record 10 evaluation episodes:

```bash
lerobot-record  \
  --robot.type=lerobot_robot_piper \
  --robot.can_name=can0 \
  --robot.id=my_awesome_piper_arm \
  --robot.cameras="{ up: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30},front: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 30}}" \
  --display_data=false \
  --dataset.repo_id=starai/eval_record-test \
  --dataset.single_task="Put lego brick into the transparent box" \
  --policy.path=outputs/train/act_piper_test/checkpoints/last/pretrained_model
  # <- Teleop optional if you want to teleoperate in between episodes \
  # --teleop.type=lerobot_teleoperator_violin \
  # --teleop.port=/dev/ttyUSB0 \
  # --teleop.id=my_awesome_leader_arm \
```

## Reference Documentation

Seeed Wiki：[How to use the SO10xArm robotic arm in Lerobot | Seeed Studio Wiki]([如何在 Lerobot 中使用 SO100/101Arm 机器人手臂 | Seeed Studio Wiki](https://wiki.seeedstudio.com/cn/lerobot_so100m/))

Huggingface Project:[Lerobot](https://github.com/huggingface/lerobot/tree/main)

Huggingface:[LeRobot](https://huggingface.co/docs/lerobot/index)

ACT or ALOHA:[Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://tonyzhaozh.github.io/aloha/)

VQ-BeT:[VQ-BeT: Behavior Generation with Latent Actions](https://sjlee.cc/vq-bet/)

Diffusion Policy:[Diffusion Policy](https://diffusion-policy.cs.columbia.edu/)

TD-MPC:[TD-MPC](https://www.nicklashansen.com/td-mpc/)
