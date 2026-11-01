# Joint, servo ID and software-name mapping

[← Documentation](./)　|　<sub>**English** / [简体中文](specifications.zh.md)</sub>

PiPER-Mate has six arm joints and one handle input. It acts as the leader for an AgileX PiPER follower.

**Contents:**

- [🔢 Joint and software-name mapping](#joint-ids-and-names)
- [📊 Follower command limits and directions](#follower-command-limits)

<a id="joint-ids-and-names"></a>

## Joint IDs and names

**Joint numbers and servo bus IDs use different numbering.** J1–J6 map to IDs 0–5; the handle is labeled J7 in the assembly and uses ID 6. Product joints 0–5 in the [homepage specifications](../README.md#product-specifications) correspond to J1–J6.

| Assembly joint | Servo ID | Python SDK | LeRobot | ROS 2 |
| --- | ---: | --- | --- | --- |
| J1 | 0 | `joint1` | `Joint_1` | `joint1` |
| J2 | 1 | `joint2` | `Joint_2` | `joint2` |
| J3 | 2 | `joint3` | `Joint_3` | `joint3` |
| J4 | 3 | `joint4` | `Joint_4` | `joint4` |
| J5 | 4 | `joint5` | `Joint_5` | `joint5` |
| J6 | 5 | `joint6` | `Joint_6` | `joint6` |
| J7 (handle) | 6 | `gripper` | `Gripper` | `gripper` |

- Keep the existing servo IDs 0–6. The assembly label `J7-ID6-RP6-U15H-M (Handle)` identifies the handle input, which controls the follower gripper.
- The Python SDK and ROS 2 include the handle channel when `gripper_exist` is enabled. LeRobot publishes action keys such as `Joint_1.pos` and `Gripper.pos`; the table lists the underlying motor names, including their exact capitalization.
- Matching names and IDs do not mean matching units, scaling or calibration. Handle conversion differs between workflows; follow the relevant software guide.

The names and IDs above come from the [Python mapping](../Python_SDK/piper_pipermate.py), [LeRobot teleoperator](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/starai_pipermate.py) and [ROS 2 driver](../ROS2_HUMBLE/src/piper_mate/piper_mate/piper_mate.py).

<a id="follower-command-limits"></a>

## Follower command limits and directions

The following angle clamps and direction signs are used by the Python SDK and the LeRobot teleoperator configuration for the six arm channels. The radian values are rounded for reference. The leader's mechanical travel is listed under [Product Specifications](../README.md#product-specifications).

| Servo ID / Python joint | Python / LeRobot angle clamp | Approximate radians | Direction |
| --- | --- | --- | --- |
| 0 / joint1 | −150° to +150° | −2.62 to +2.62 rad | Reverse |
| 1 / joint2 | 0° to 180° | 0 to 3.14 rad | Forward |
| 2 / joint3 | −170° to 0° | −2.97 to 0 rad | Forward |
| 3 / joint4 | −100° to +100° | −1.75 to +1.75 rad | Reverse |
| 4 / joint5 | −70° to +70° | −1.22 to +1.22 rad | Forward |
| 5 / joint6 | −120° to +120° | −2.09 to +2.09 rad | Reverse |

Sources: [Python mapping](../Python_SDK/piper_pipermate.py) and [LeRobot configuration](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py). Do not enlarge the software limits to match the leader's mechanical travel without checking the follower.

For the BOM, CAD assembly, drawings and printing resources, see [Hardware Resources](../hardware/README.md). Continue with the [Python SDK](../Python_SDK/README.md), [LeRobot](../Lerobot/README.md) or [ROS 2](../ROS2_HUMBLE/README.md) guide for your workflow.
