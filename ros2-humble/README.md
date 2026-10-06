# ROS 2 Humble teleoperation

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

**On this page:**

- [📦 Build the workspace](#build)
- [🦾 Start the leader](#leader)
- [▶️ Start the follower](#follower)
- [🔎 Inspect topics and source](#topics)

<a id="build"></a>

## 📦 Build the workspace

Install ROS 2 Humble on Ubuntu 22.04 first, then complete [connection setup](../docs/README.md#connect). Run from the repository root. Initialize rosdep with `sudo rosdep init` only if it has not already been initialized.

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

Use the system Python 3.10 supplied with Ubuntu 22.04. The command above installs colcon in the same environment as the SDKs; keep that environment active for builds and both terminals. Native ROS execution has not been tested in this restructuring.

<a id="leader"></a>

## 🦾 Start the leader

Open terminal 1 at the repository root:

```bash
source /opt/ros/humble/setup.bash
source .venv-ros/bin/activate
source ros2-humble/install/setup.bash
ros2 run piper_mate driver --ros-args \
  -p port:=/dev/ttyUSB0 -p auto_enable:=false -p gripper_exist:=true
```

`auto_enable:=false` leaves leader torque disabled. The driver unlocks servos and resets multi-turn counts during initialization. Support the arm and check the published joint values before starting the follower.

<a id="follower"></a>

## ▶️ Start the follower

In terminal 2 at the repository root, after completing the [motion checks](../docs/README.md#operation):

```bash
source /opt/ros/humble/setup.bash
source .venv-ros/bin/activate
source ros2-humble/install/setup.bash
bash can_activate.sh can0 1000000
ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:=can0 -p auto_enable:=true \
  -p gripper_exist:=true -p gripper_val_mutiple:=2
```

`auto_enable:=true` can enable follower motion. `gripper_val_mutiple` is the existing parameter spelling; its value scales the gripper command. Review this multiplier for your gripper rather than assuming it matches the Python or LeRobot path.

<a id="topics"></a>

## 🔎 Inspect topics and source

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_ctrl_single
ros2 topic echo /arm_status
```

| Component | Source / interface |
| --- | --- |
| Leader executable `driver` | [piper_mate.py](src/piper_mate/piper_mate/piper_mate.py); publishes `sensor_msgs/JointState` on `joint_ctrl_single` |
| Follower executable `piper_single_ctrl` | [piper_ctrl_single_node.py](src/piper/piper/piper_ctrl_single_node.py); subscribes to `joint_ctrl_single` |
| Status | `arm_status`, `joint_states_single`, `joint_states_feedback` |

The `_new.py` controller is not the executable selected by this guide. Keep package names unchanged. This workspace provides teleoperation nodes; it does not contain a complete PiPER-Mate URDF, MoveIt 2 configuration or Gazebo simulation. For follower resources, see [AgileX PiPER ROS](https://github.com/agilexrobotics/piper_ros/tree/humble).
