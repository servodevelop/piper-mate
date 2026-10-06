# PiPER-Mate hardware

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

<p align="center">
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-01.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-01.webp" alt="PiPER-Mate view 1" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-02.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-02.webp" alt="PiPER-Mate view 2" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp" alt="PiPER-Mate view 3" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-04.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-04.webp" alt="PiPER-Mate view 4" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-05.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-05.webp" alt="PiPER-Mate view 5" width="18%"></a>
</p>

**On this page:**

- [📂 Download resources](#resources)
- [🔧 Specifications and command limits](#specifications)
- [📋 Assembly BOM](#bill-of-materials)
- [🖨️ Print projects](#printing)
- [📐 Dimension drawing](#dimensions)

<a id="resources"></a>

## 📂 Download resources

| Resource | Files | Use / status |
| --- | --- | --- |
| Assembly model | [piper-mate.step](step/assembly/piper-mate.step) | Neutral CAD reference; revision alignment with individual parts remains unverified |
| Part models | [step/parts/](step/parts/) | 12 STEP files; BOM part names link directly to their models |
| Print projects | [printing/3mf/](printing/3mf/) | Published profile and 09-01 reference; compare settings below |
| Dimension drawing | [PDF](drawings/piper-mate.pdf) · [DWG](drawings/piper-mate.dwg) | Drawing dated 2026-08-14; no assembly sequence |
| Editable source | [SolidWorks ZIP](cad-source/piper-mate-solidworks.zip) | 20 native files with original filenames; assembly dependencies not verified in SolidWorks |
| Build preparation | [Assembly and printing guide](assembly-guide/README.md) | Resource selection and checks before building; full assembly tutorial pending |

The native bundle retains alternate part names to preserve possible assembly references. STEP is the portable starting point; the bundle is not a verified Pack and Go export. Backup files and rendering-session files are excluded.

<a id="specifications"></a>

## 🔧 Specifications and command limits

| Specification | PiPER-Mate leader |
| --- | --- |
| Reach / DoF | 400 mm / 6 arm joints + 1 trigger |
| Joint servos | RP6-U15H-M × 7 (assembly BOM) |
| Position sensing | 12-bit absolute encoder |
| Communication | UART bus, UC-01 USB interface; examples use 1,000,000 baud |
| Leader power | 12 V, 3 A, DC5521 |
| Arm weight | 500 g (product specification) |
| Follower connection | AgileX PiPER via USB-to-CAN, 1 Mbit/s |

The [product specification](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/) numbers joints from 0. The table pairs these with the software's 1-based names. Mechanical travel and follower command limits serve different purposes.

| Servo ID / joint | Product travel | Python / LeRobot follower command clamp | Direction |
| --- | --- | --- | --- |
| 0 / joint1 | −154° to +154° | −150° to +150° | Reverse |
| 1 / joint2 | 0° to 195° | 0° to 180° | Forward |
| 2 / joint3 | −175° to 0° | −170° to 0° | Forward |
| 3 / joint4 | −102° to +102° | −100° to +100° | Reverse |
| 4 / joint5 | −142° to +142° | −70° to +70° | Forward |
| 5 / joint6 | −120° to +120° | −120° to +120° | Reverse |

Source: [Python mapping](../python-sdk/piper_pipermate.py) and [LeRobot mapping](../lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py). Trigger conversion differs between workflows; see their guides. Do not enlarge software limits to match the leader's mechanical travel without checking the follower.

<a id="bill-of-materials"></a>

## 📋 Assembly BOM

These **19 installed-item entries** preserve the model numbers, quantities and cable orientations in the assembly BOM. Servos are listed first. Printed-part filenames use the names below; original BOM identifiers remain in the description column.

| No. | Name | Image | Specification / description | Qty. | Notes |
| :---: | --- | :---: | --- | :---: | --- |
| 1 | RP6-U15H-M | Pending | Coreless UART bus servo | 7 | Six arm joints + trigger |
| 2 | [base](step/parts/base.step) | Pending | PLA; Piper mate_ID0 | 1 | Base |
| 3 | [link1](step/parts/link1.step) | Pending | PLA; Piper mate_ID1 | 1 | Link 1 |
| 4 | [link2](step/parts/link2.step) | Pending | PLA; Piper mate_ID2 | 1 | Link 2 |
| 5 | [link3](step/parts/link3.step) | Pending | PLA; Piper mate_ID3 | 1 | Link 3 |
| 6 | [link4](step/parts/link4.step) | Pending | PLA; Piper mate_ID4 | 1 | Link 4 |
| 7 | [link5](step/parts/link5.step) | Pending | PLA; Piper mate_ID5 | 1 | Link 5 |
| 8 | [trigger](step/parts/trigger.step) | Pending | PLA; Piper mate_ID6-0 | 1 | Handle / trigger |
| 9 | [finger-ring-1](step/parts/finger-ring-1.step) | Pending | PLA; Piper mate_ID6-1 | 1 | First finger ring |
| 10 | [finger-ring-2](step/parts/finger-ring-2.step) | Pending | PLA; Piper mate_ID6-2 | 1 | Second finger ring |
| 11 | Servo cable | Pending | A2007H03 × 2; 100 mm | 4 | Reversed connector orientation |
| 12 | Servo cable | Pending | A2007H03 × 2; 140 mm | 1 | Reversed connector orientation |
| 13 | Servo cable | Pending | A2007H03 × 2; 250 mm | 1 | Reversed connector orientation |
| 14 | Adapter cable | Pending | A2007H03 to PH2.0; 100 mm | 1 | Same connector orientation |
| 15 | Socket screw | Pending | PM M1.6 × 4 mm; black; grade 12.9 | 8 | Installed hardware |
| 16 | Phillips screw | Pending | PM M1.6 × 4 mm | 54 | Installed hardware |
| 17 | Countersunk screw | Pending | M3 × 10; black; Phillips | 1 | Installed hardware |
| 18 | Locknut | Pending | M3; black | 1 | Installed hardware |
| 19 | UC-01 | Pending | PCBA with DC5521 female power lead | 1 | 0.75 mm² lead; 150 mm length |

The UC-01 cover, front case and mount are available in [step/parts/](step/parts/) but are not separately quantified in this BOM. Confirm their fit and required quantities for your configuration. The assembly model is not an additional BOM item.

Useful accessories listed separately in the source include low-strength purple 222 threadlocker, a 1 m USB-A to USB-C cable and two 4-inch F-clamps. Packaging, spare screws/cables and a separately listed bare UC-01 are not added to installed quantities. Confirm the actual kit contents with your order.

<a id="printing"></a>

## 🖨️ Print projects

| Project | Printer / material | Stored global settings | Status |
| --- | --- | --- | --- |
| [piper-mate.3mf](printing/3mf/piper-mate.3mf) | Bambu Lab P1S, 0.4 mm nozzle, PLA | 0.2 mm layers; 30% infill; 5 walls | Existing repository print profile |
| [piper-mate-09-01.3mf](printing/3mf/piper-mate-09-01.3mf) | Bambu Lab P1S, 0.4 mm nozzle, PLA | 0.2 mm layers; 15% infill; 4 walls | Source marked “9.1 update”; fit and strength not verified |

Settings are read from each project's metadata, not a print recommendation. Per-object overrides, supports and orientation must be reviewed in the slicer. The 09-01 project contains a different UC-01 enclosure arrangement; do not mix revisions without a fit check. Individual STL files are not supplied. The separately named handle project contains multiple arm parts and is not presented as a handle-only print.

<a id="dimensions"></a>

## 📐 Dimension drawing

[![PiPER-Mate dimension drawing](images/dimensions.png)](drawings/piper-mate.pdf)

Open the [PDF](drawings/piper-mate.pdf) for dimensions or the [DWG](drawings/piper-mate.dwg) for CAD editing. Check revision agreement with the selected model before fabrication.
