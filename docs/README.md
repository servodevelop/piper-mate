# Get started with PiPER-Mate

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

**On this page:**

- [🧰 Prepare your setup](#prepare)
- [🔌 Connect USB and CAN](#connect)
- [🚀 Choose a workflow](#choose)
- [🛑 First motion and stopping](#operation)
- [🔎 Troubleshooting](#troubleshooting)
- [🧪 Validation status](#validation)

<a id="prepare"></a>

## 🧰 Prepare your setup

Use Ubuntu 22.04 with a PiPER-Mate leader, an AgileX PiPER follower, a Linux SocketCAN-compatible USB-to-CAN adapter and separate suitable power supplies. ROS 2 Humble is required only for the ROS workflow. Cameras and a training-capable GPU are optional for initial teleoperation.

Mount both arms securely and clear their motion space. Confirm the follower firmware and SDK combination against [AgileX's instructions](https://github.com/agilexrobotics/piper_sdk). These hardware workflows target Linux; macOS control has not been validated.

```text
PiPER-Mate leader -- UC-01 -- USB -------- Ubuntu 22.04 computer
                                              |
AgileX PiPER follower -- CAN -- USB-CAN --------+

Leader: 12 V / 3 A supply
Follower: its own manufacturer-specified supply
```
For a kit, first read the [hardware BOM](../hardware/README.md#bill-of-materials) and [build preparation](../hardware/assembly-guide/README.md).

<a id="connect"></a>

## 🔌 Connect USB and CAN

Clone the repository once. The following commands run from its root:

```bash
git clone https://github.com/servodevelop/piper-mate.git
cd piper-mate
```

Identify the leader serial device, then grant your Linux account serial access:

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB*
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. Prefer a stable `/dev/serial/by-id/...` path when available. Do not assume that every `/dev/ttyUSB*` device is the leader.

For the follower, activate the intended CAN adapter:

```bash
sudo apt update
sudo apt install can-utils ethtool
bash find_all_can_port.sh
bash can_activate.sh can0 1000000
ip -details link show can0
```

Check the detected adapter before changing its name. For multiple CAN devices, inspect [can_config.sh](../can_config.sh) and [can_muti_activate.sh](../can_muti_activate.sh); the latter's historical filename is preserved. These scripts change Linux network interfaces and may request sudo.

<a id="choose"></a>

## 🚀 Choose a workflow

| Your task | Start here | Current scope |
| --- | --- | --- |
| Direct leader-to-follower control | [Python SDK](../python-sdk/README.md) | One leader and one PiPER; configurable ports in `main()` |
| ROS topic integration | [ROS 2 Humble](../ros2-humble/README.md) | Leader publishes `joint_ctrl_single`; follower subscribes |
| Demonstrations and ACT training | [LeRobot](../lerobot/README.md) | Local adapters and command examples; hardware validation pending |

Only run one control workflow at a time. Complete connection, zero-position and direction checks before enabling follower motion.

<a id="operation"></a>

## 🛑 First motion and stopping

The examples can unlock the leader, reset multi-turn counts and send follower commands during startup. Support the leader before connecting, and verify physical alignment and servo IDs 0–6. A multi-turn reset is not a complete calibration procedure.

Start without a payload and with adequate clearance. The current Python and LeRobot code includes speed-control values of 100; review the follower's documented behavior before use. Do not treat a lower loop frequency as a speed limit.

`Ctrl+C` requests application shutdown. It does **not** guarantee follower motor disable or a hardware emergency stop, particularly after communication loss. Use the follower manufacturer's stop procedure and a reachable physical power/stop control. Software joint clamps do not detect collisions.

<a id="troubleshooting"></a>

## 🔎 Troubleshooting

| Symptom | Check |
| --- | --- |
| Serial device missing | USB data cable, UC-01 power, `lsusb`, `/dev/serial/by-id/` and kernel logs |
| Permission denied | Serial device ownership and `dialout` membership after logging back in |
| CAN absent or BUS-OFF | Correct adapter, 1 Mbit/s bitrate, follower power, CAN wiring and termination |
| Unexpected direction / jump | Stop motion; check physical zero, IDs and [mapping](../hardware/README.md#specifications) |
| LeRobot cannot load the device | Activate the intended environment and install both local adapter packages |
| Camera fails | Run `lerobot-find-cameras opencv`; verify device path, bandwidth and another application using the camera |

When reporting a problem, include repository commit, OS, package versions, follower firmware, connection diagram and logs with personal paths removed. Use [GitHub Issues](https://github.com/servodevelop/piper-mate/issues).

<a id="validation"></a>

## 🧪 Validation status

| Area | Evidence available | Still required |
| --- | --- | --- |
| Documentation | Local links, bilingual structure and source cross-checks | Hardware-specific confirmation of the procedures |
| Control software | Python source and shell syntax checks | Ubuntu 22.04 / Humble execution, dependency matrix and hardware smoke tests |
| Performance | Python loop target is 100 Hz | Measured sustained rate and latency |
| Hardware resources | Files inspected; imported bytes compared with sources | CAD assembly reopening, cross-revision fit, printing and load checks |
| LeRobot | Plugin registration and CLI names checked against the documented fork snapshot | Calibration implementation, dataset / train / replay / inference runs |

Hardware execution, CAN timing and GPU training results are not yet published for this revision. A matching source layout is not a tested hardware combination.
