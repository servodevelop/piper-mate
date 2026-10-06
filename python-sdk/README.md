# Python SDK teleoperation

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

**On this page:**

- [📦 Install](#setup)
- [⚙️ Configure](#configure)
- [▶️ Run](#run)
- [↔️ Mapping and limitations](#mapping)

<a id="setup"></a>

## 📦 Install

Complete [USB / CAN setup](../docs/README.md#connect). From the repository root, create a virtual environment and install dependencies:

```bash
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python-sdk/requirements.txt
```

The dependency list reflects source imports and the existing SDK requirement. It is not a hardware-qualified lockfile. Record installed versions with `python -m pip freeze` when reporting a test.

<a id="configure"></a>

## ⚙️ Configure

Edit these existing constants in [piper_pipermate.py](piper_pipermate.py), inside `main()`:

```python
PIPERMATE_PORT = "/dev/ttyUSB0"
PIPER_CAN_NAME = "can0"
GRIPPER_EXIST = True
UPDATE_RATE = 100.0
```

There are no command-line flags for these settings. `UPDATE_RATE` determines a sleep interval after each read/write cycle, so actual frequency is lower than the target when work takes time. The script unlocks the leader and clears multi-turn counts at startup.

<a id="run"></a>

## ▶️ Run

Read [first motion and stopping](../docs/README.md#operation) before running:

```bash
python python-sdk/piper_pipermate.py
```

The script opens the leader serial port and PiPER CAN interface and sends joint commands. It does not explicitly call `EnablePiper()`; check the follower enable procedure for your SDK / firmware. `Ctrl+C` exits the loop and attempts to close connections, but does not explicitly disable follower motors.

<a id="mapping"></a>

## ↔️ Mapping and limitations

Servo IDs 0–5 map to `joint1`–`joint6`, with IDs 0, 3 and 5 reversed before the [joint clamps](../hardware/README.md#specifications). The follower receives 0.001-degree units.

ID 6 supplies the trigger. Python converts 0–45° into 0–0.08 m and then micrometres for the gripper command. This conversion has no explicit clamp in the Python script; verify trigger travel before use.

Serial and OS errors can break the main loop; other read errors return an empty state and the loop can continue. Disconnection behavior is not a tested fail-safe. See [validation status](../docs/README.md#validation).
