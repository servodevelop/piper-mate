# PiPER Mate and Piper integrated control

[← Parent](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

Read PiPER Mate leader angles over USB serial and control the Piper follower over CAN. This directory provides three controller choices: teleoperation only, teleoperation with fixed damping, and teleoperation with gripper feedback.

**On this page:**

- [🧭 Choose a script](#overview)
- [📦 Install dependencies](#setup)
- [🔌 Configure connections](#quick-start)
- [▶️ Run the three controllers](#run)
- [↔️ Shared mapping and control mode](#mapping)
- [🛠️ CAN utility scripts](#scripts)
- [🔧 Troubleshooting](#troubleshooting)
- [🛑 Operation and stopping](#safety)
- [📄 License](#license)
- [🤝 Contributing](#contributing)
- [📬 Contact](#contact)

<a id="overview"></a>

## 🧭 Choose a script

Each script independently provides leader-to-follower teleoperation. They differ in how leader damping is generated. **Run one controller at a time; do not start a separate teleoperation process first.**

<a id="features"></a>

| Script | Leader damping | Use |
| --- | --- | --- |
| [piper_pipermate.py](piper_pipermate.py) | Unlocks at startup; no load feedback | Basic position teleoperation |
| [mate_damping.py](mate_damping.py) | Fixed six-joint damping, independent of follower load | Teleoperation with constant resistance |
| [piper_gripper_feedback.py](piper_gripper_feedback.py) | Load-dependent gripper damping; heavier closing, lighter opening | Teleoperation with gripper-load feedback |

Complete installation and connection setup below, then choose a controller in the run section. Loop frequencies are software targets; achieved rate, tracking accuracy and damping feel require hardware validation.

<a id="setup"></a>

## 📦 Install dependencies

### Hardware

- PiPER Mate leader connected through USB serial.
- Piper follower connected through CAN.
- A SocketCAN-compatible USB–CAN adapter.
- Linux, for example Ubuntu 22.04; validate your SDK/firmware combination.

### Software dependencies

Run all commands below from the **repository root**. Install CAN utilities and use a Python virtual environment:

```bash
sudo apt update
sudo apt install ethtool can-utils python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python-sdk/requirements.txt
```

Dependencies include `pyserial`, `fashionstar-uart-sdk`, `piper-sdk`, `python-can` and `scipy`. This is not a hardware-qualified lockfile; record the environment with `python -m pip freeze` when reporting tests.

<a id="quick-start"></a>

## 🔌 Configure connections

All three controllers use USB serial and CAN. This example configures interface `can0` at `1000000` bit/s:

```bash
bash find_all_can_port.sh
bash can_activate.sh can0 1000000
```

[USB/CAN setup and permissions](../docs/README.md#connect). Enable the follower through your SDK/firmware procedure before running any controller below; the scripts do not perform that enable procedure automatically.

<a id="run"></a>

## ▶️ Run the three controllers

Run commands from the **repository root**. The three entries are alternatives; do not run them together.

<a id="teleoperation"></a>

### 1. `piper_pipermate.py` — Teleoperation only

**Purpose**: read six leader joints and an optional gripper, then send follower position targets. No follower-load damping is applied to the leader.

**Run**:

```bash
python python-sdk/piper_pipermate.py
```

<a id="configure"></a>

**Configure**: edit these values in the script's `main()`; there are no corresponding CLI flags:

```python
PIPERMATE_PORT = "/dev/ttyUSB0"
PIPER_CAN_NAME = "can0"
GRIPPER_EXIST = True
UPDATE_RATE = 100.0
```

These specify the leader port, follower CAN interface, gripper switch and target rate in Hz. `UPDATE_RATE` sets a sleep after each read/write cycle; actual cycle time also includes the work.

<a id="damping"></a>

### 2. `mate_damping.py` — Teleoperation with fixed damping

**Purpose**: [mate_damping.py](mate_damping.py) adds **fixed damping** to six leader joints while retaining seven-axis position teleoperation. It does not adjust damping from follower torque and is not load feedback.

**Run**:

```bash
python python-sdk/mate_damping.py
```

**Configure**: Edit the port, CAN name, gripper switch and target rate in this script's `main()`, using the same four names shown above. Edit fixed damping at the initialization `set_damping()` calls; current settings are:

| Leader ID | Part | Fixed setting |
| --- | --- | --- |
| 0 | J1 | `set_damping(..., power=0)` |
| 1–2 | J2–J3 | 200 mW |
| 3–5 | J4–J6 | 100 mW |
| 6 | Gripper | No gripper damping applied |

Do not add `--feedback-enable`; this script has no corresponding CLI option. Values were checked against source; feel was not hardware-tested in this documentation change.

<a id="feedback"></a>

### 3. `piper_gripper_feedback.py` — Teleoperation with gripper feedback

**Purpose**: [piper_gripper_feedback.py](piper_gripper_feedback.py) integrates seven-axis teleoperation with **gripper-only load feedback** in one file. Leader opening commands follower opening; only leader gripper ID 6 receives damping. It has no six-joint feedback, URDF or global HOLD.

**Run**:

```bash
# Teleoperation + actual gripper damping
python python-sdk/piper_gripper_feedback.py --feedback-enable

# Set leader port, follower CAN interface and loop rate
python python-sdk/piper_gripper_feedback.py --port /dev/ttyUSB0 --can can0 --rate 30 --feedback-enable

# Teleoperation + calculation only; no gripper damping output
python python-sdk/piper_gripper_feedback.py
```

**Omitting `--feedback-enable` still teleoperates the follower**; it only suppresses gripper damping on the leader.

Processing: read Piper gripper CAN torque feedback → filter and deadzone/load confirmation → select a heavier closing or lighter opening target from leader motion → power filtering and slew limits → write leader gripper damping. Direction changes preserve torque filter history.

**Configure**: feel parameters are at the top of the file; restart after editing:

| Parameter | Current value | Purpose |
| --- | --- | --- |
| `DEFAULT_POWER_MW` | 1500 mW | Total closing damping cap |
| `GRIPPER_BASE_POWER_MW` | 20 mW | Baseline damping in both directions |
| `GRIPPER_DEADZONE_NM` | 0.15 Nm | Base load threshold |
| `GRIPPER_FULL_SCALE_NM` | 0.30 Nm | Excess torque for the full damping increment |
| `GRIPPER_OPEN_RATIO` | 0.20 | Opening fraction of the load-dependent increment |
| `GRIPPER_OPEN_MAX_POWER_MW` | 240 mW | Opening target cap |

Logs show states such as `HEAVY_LOAD` / `LIGHT_LOAD`, raw/filtered torque, effective deadzone, target and calculated output power. Nm describes feedback torque; mW is a leader SDK damping parameter. They are not interchangeable. Use the current script values when tuning feel. Load estimation, communication and physical feel remain uncalibrated on hardware.

<a id="mapping"></a>

## ↔️ Shared mapping and control mode

### Joint-angle conversion

All three controllers use this position mapping. IDs 0–5 correspond to J1–J6; transmitted joint units are 0.001°.

| Joint | Angle limits | Radian limits (approx.) | Direction |
| --- | --- | --- | --- |
| Joint1 | -150° ~ 150° | -2.6179 ~ 2.6179 | Reversed |
| Joint2 | 0° ~ 180° | 0 ~ 3.14 | Forward |
| Joint3 | -170° ~ 0° | -2.967 ~ 0 | Forward |
| Joint4 | -100° ~ 100° | -1.745 ~ 1.745 | Reversed |
| Joint5 | -70° ~ 70° | -1.22 ~ 1.22 | Forward |
| Joint6 | -120° ~ 120° | -2.09439 ~ 2.09439 | Reversed |

### Gripper control

ID 6 uses a 0–45° to 0–0.08 m conversion, then converts to micrometres for the gripper command. This conversion has no additional explicit clamp; check actual trigger travel before use. Gripper position commands and their fixed torque field do not implement gripper-load feedback.

### Control mode

All three controllers pass `ctrl_mode=0x01`, `move_mode=0x01`, `move_spd_rate_ctrl=100` and `is_mit_mode=0xAD` to the Piper SDK. Interpret these values against the matching SDK/firmware.

<a id="scripts"></a>

## 🛠️ CAN utility scripts

These scripts are in the repository root.

| Script | Purpose |
| --- | --- |
| [can_activate.sh](../can_activate.sh) | Activate one interface, set bitrate, rename and match USB hardware addresses |
| [can_config.sh](../can_config.sh) | Single/multiple CAN configuration, USB port mapping and bitrate setup |
| [can_muti_activate.sh](../can_muti_activate.sh) | Multiple-device configuration, duplicate-name checks and interactive confirmation |
| [find_all_can_port.sh](../find_all_can_port.sh) | List CAN interfaces and USB port information |

<a id="troubleshooting"></a>

## 🔧 Troubleshooting

### Common issues

| Issue | Check |
| --- | --- |
| CAN interface missing | Check adapter connection, run `bash find_all_can_port.sh`, and verify USB mapping |
| Serial permission denied | Check device group ownership and `dialout` membership in the connection guide |
| PiPER Mate connection failure | Check `/dev/ttyUSB0`, device permissions and Fashion Star SDK installation |
| Follower does not track | Check CAN name, bitrate, follower enable state and firmware |

### Debug output

The original teleoperation script prints mapped joint/gripper targets and errors. These values are not independently measured follower positions; the original script does not report achieved frequency. Serial or OS errors can exit the loop; other read errors may return an empty state and continue.

[Connection and troubleshooting guide](../docs/README.md#connect).

<a id="safety"></a>

## 🛑 Operation and stopping

1. **Before motion**: check leader/follower starting poses, connections and available workspace.
2. **Stopping**: `Ctrl+C` requests exit and connection cleanup; it is not a hardware emergency stop. Scripts do not automatically disable or home the follower.
3. **Leader damping**: teleoperation only unlocks the leader at startup; the fixed-damping and gripper-feedback scripts change leader resistance.
4. **Joint limits**: soft limits constrain six joint targets; they do not guarantee collision, pinch or disconnection protection.
5. **Run one controller**: these scripts share serial/CAN connections and must not run together.

Hardware communication, stopping, tracking accuracy and feedback feel have not completed a common hardware validation procedure.

[First motion, stopping and validation status](../docs/README.md#operation).

<a id="license"></a>

## 📄 License

A project-wide software/hardware license is pending. See [LICENSE.md](../LICENSE.md) for existing component scopes. The older README statement of a blanket MIT license is not a project-wide license grant.

<a id="contributing"></a>

## 🤝 Contributing

Issues and pull requests are welcome. For control reports, include SDK/firmware versions, command, parameters and logs, and state whether hardware was tested.

<a id="contact"></a>

## 📬 Contact

[Contact maintainers through Issues](https://github.com/servodevelop/piper-mate/issues).
