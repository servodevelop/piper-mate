# PiPER-Mate hardware

[← PiPER-Mate](../README.md)　|　<sub>**English** / [简体中文](README.zh.md)</sub>

<p align="center">
  <a href="images/piper-mate-main-image-01.webp"><img src="images/piper-mate-main-image-01.webp" alt="PiPER-Mate product view 1" width="15%"></a>
  <a href="images/piper-mate-main-image-02.webp"><img src="images/piper-mate-main-image-02.webp" alt="PiPER-Mate product view 2" width="15%"></a>
  <a href="images/piper-mate-main-image-03.webp"><img src="images/piper-mate-main-image-03.webp" alt="PiPER-Mate product view 3" width="15%"></a>
  <a href="images/piper-mate-main-image-04.webp"><img src="images/piper-mate-main-image-04.webp" alt="PiPER-Mate product view 4" width="15%"></a>
  <a href="images/piper-mate-main-image-05.webp"><img src="images/piper-mate-main-image-05.webp" alt="PiPER-Mate product view 5" width="15%"></a>
</p>

**Contents:**

- [📂 Resources](#resources)
- [🔧 Specifications](#specifications)
- [📋 Bill of Materials](#bill-of-materials)
- [📐 Dimensions](#dimensions)

<a id="resources"></a>

## Resources

| Directory | Description | Status |
| --- | --- | --- |
| [step/](step/) | Complete assembly in assembly/; 11 part models in parts/ | ✅ Complete |
| [stl/](stl/) | [Download all STL files (11 parts)](stl/downloads/piper-mate-stl-2026-10-27.zip?raw=true) · [Browse individual STL files](stl/) | ✅ Complete |
| [drawings/](drawings/) | Dimension drawing in PDF and DWG | ✅ Complete |

The assembly model is a neutral CAD reference; its revision alignment with the individual part files has not been fully verified. The STEP assembly and all 11 part files are based on the supplied 2026-10-27 release folder. Geometry is unchanged; only the PRODUCT names were normalized to the identifiers below.

The assembly tree labels are `piper-mate-base`, `piper-mate-link 1`–`5`, `piper-mate-handle`, and `piper-mate-finger-ring-right` / `left`. The corresponding part files use the same identifiers, with the space dropped in `piper-mate-link1`–`5`.

The portable print entry is [stl/](stl/): 11 individual binary STL files plus a ZIP containing the same 11 files. Every loose STL is byte-identical to its ZIP entry.

<a id="specifications"></a>

## Specifications

| Specification | PiPER-Mate leader |
| --- | --- |
| Reach / DoF | 400 mm / 6 arm joints + 1 trigger |
| Joint servos | RP6-U15H-M × 7 |
| Position sensing | 12-bit absolute encoder |
| Communication | UART bus, UC-01 USB interface; examples use 1,000,000 baud |
| Power | 12 V, 3 A, DC5521 |
| Arm weight | 500 g |
| Follower connection | AgileX PiPER over USB-to-CAN, 1 Mbit/s |

The product specification numbers joints from 0. The table below pairs those numbers with the 1-based joint names used by the software. Mechanical travel and follower command limits serve different purposes.

| Servo ID / joint | Product travel | Python / LeRobot follower command clamp | Direction |
| --- | --- | --- | --- |
| 0 / joint1 | −154° to +154° | −150° to +150° | Reverse |
| 1 / joint2 | 0° to 195° | 0° to 180° | Forward |
| 2 / joint3 | −175° to 0° | −170° to 0° | Forward |
| 3 / joint4 | −102° to +102° | −100° to +100° | Reverse |
| 4 / joint5 | −142° to +142° | −70° to +70° | Forward |
| 5 / joint6 | −120° to +120° | −120° to +120° | Reverse |

Sources: [Python mapping](../Python_SDK/piper_pipermate.py) and [LeRobot mapping](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py). Trigger conversion differs between workflows; see their guides. Do not enlarge the software limits to match the leader's mechanical travel without checking the follower.

<a id="bill-of-materials"></a>

## Bill of Materials

The table lists **31 items**: rows 1–21 are installed items, rows 22–31 are kit accessories / packaging. Servos are listed first, and printed-part identifiers match the STEP and STL filenames. The description column keeps the original BOM identifier.

Print settings are not recorded here: take the STL from [stl/](stl/) and slice it for your own printer, because layer height, wall count and infill depend on the machine and filament used.

Fastener images are illustrative; some show a different length or thread of the same head type. Use the specification column for purchasing.

| No. | Part Name | Model / Specification / Part Identifier | Image | Description | Quantity |
| :---: | --- | --- | :---: | --- | :---: |
| 1 | UART bus servo | RP6-U15H-M | <a href="images/rp6-u15h-m.png"><img src="images/rp6-u15h-m.png" alt="RP6-U15H-M" width="90"></a> | Coreless motor; 12-bit absolute encoder; six arm joints plus trigger | 7 |
| 2 | piper-mate-base | [piper-mate-base](step/parts/piper-mate-base.step) | <a href="images/base.png"><img src="images/base.png" alt="base" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID0` | 1 |
| 3 | piper-mate-link 1 | [piper-mate-link1](step/parts/piper-mate-link1.step) | <a href="images/link1.png"><img src="images/link1.png" alt="link1" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID1` | 1 |
| 4 | piper-mate-link 2 | [piper-mate-link2](step/parts/piper-mate-link2.step) | <a href="images/link2.png"><img src="images/link2.png" alt="link2" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID2` | 1 |
| 5 | piper-mate-link 3 | [piper-mate-link3](step/parts/piper-mate-link3.step) | <a href="images/link3.png"><img src="images/link3.png" alt="link3" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID3` | 1 |
| 6 | piper-mate-link 4 | [piper-mate-link4](step/parts/piper-mate-link4.step) | <a href="images/link4.png"><img src="images/link4.png" alt="link4" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID4` | 1 |
| 7 | piper-mate-link 5 | [piper-mate-link5](step/parts/piper-mate-link5.step) | <a href="images/link5.png"><img src="images/link5.png" alt="link5" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID5` | 1 |
| 8 | piper-mate-handle | [piper-mate-handle](step/parts/piper-mate-handle.step) | <a href="images/trigger.png"><img src="images/trigger.png" alt="handle" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID6-0` | 1 |
| 9 | piper-mate-finger-ring-right | [piper-mate-finger-ring-right](step/parts/piper-mate-finger-ring-right.step) | <a href="images/finger-ring-1.png"><img src="images/finger-ring-1.png" alt="right finger ring" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID6-1` | 1 |
| 10 | piper-mate-finger-ring-left | [piper-mate-finger-ring-left](step/parts/piper-mate-finger-ring-left.step) | <a href="images/finger-ring-2.png"><img src="images/finger-ring-2.png" alt="left finger ring" width="90"></a> | Ivory, PLA 3D printed part; BOM ID `Piper mate_ID6-2` | 1 |
| 11 | piper-mate-uc01-front-case | [piper-mate-uc01-front-case](step/parts/piper-mate-uc01-front-case.step) | <a href="images/uc01-front-case.png"><img src="images/uc01-front-case.png" alt="UC-01 front case" width="90"></a> | Ivory, PLA 3D printed part; UC-01 enclosure front case | 1 |
| 12 | piper-mate-uc01-mount | [piper-mate-uc01-mount](step/parts/piper-mate-uc01-mount.step) | <a href="images/uc01-mount.png"><img src="images/uc01-mount.png" alt="UC-01 mount" width="90"></a> | Ivory, PLA 3D printed part; UC-01 mounting plate | 1 |
| 13 | Servo cable | A2007H03 × 2<br>100 mm | Pending | Reversed connector orientation between ends | 4 |
| 14 | Servo cable | A2007H03 × 2<br>140 mm | Pending | Reversed connector orientation between ends | 1 |
| 15 | Servo cable | A2007H03 × 2<br>250 mm | Pending | Reversed connector orientation between ends | 1 |
| 16 | Adapter cable | A2007H03 to PH2.0<br>100 mm | Pending | Same connector orientation at both ends | 1 |
| 17 | Hexagon socket head cap screw | PM M1.6 × 4 mm | <a href="images/fasteners/m2-8-socket-head-cap-screw.png"><img src="images/fasteners/m2-8-socket-head-cap-screw.png" alt="Hexagon socket head cap screw" width="90"></a> | Black finish, property class 12.9 | 8 |
| 18 | Phillips pan head machine screw | PM M1.6 × 4 mm | <a href="images/fasteners/m2-4-5-phillips-pan-head-screw.png"><img src="images/fasteners/m2-4-5-phillips-pan-head-screw.png" alt="Phillips pan head machine screw" width="90"></a> | Black finish | 54 |
| 19 | Phillips countersunk head machine screw | M3 × 10 | Pending | Black finish | 1 |
| 20 | Locknut | M3 | <a href="images/fasteners/m3-nut.png"><img src="images/fasteners/m3-nut.png" alt="M3 locknut" width="90"></a> | Black finish | 1 |
| 21 | TTL/USB servo adapter | UC-01 | <a href="images/uc-01.webp"><img src="images/uc-01.webp" alt="UC-01" width="90"></a> | PCBA with DC5521 female power lead; 0.75 mm² lead, 150 mm long | 1 |
| 22 | White box | 160 × 125 × 60 mm | Pending | Small white packaging box | 1 |
| 23 | Sealing bag | 120 × 80 mm | Pending | Holds the threadlocker; label reads “Threadlocker 1pcs / 螺丝胶” | 1 |
| 24 | Threadlocker | Purple 222, low strength | Pending | — | 1 |
| 25 | Spare power cable | XT30, male, 10 cm | Pending | — | 1 |
| 26 | USB cable | USB-A to USB-C, 1 m, black, 1 A | Pending | — | 1 |
| 27 | F-clamp | 4-inch F-clamp | Pending | — | 2 |
| 28 | UC-01 (bare board) | UC-01 without power cable | Pending | — | 1 |
| 29 | Servo cable | A2007H03 × 2<br>250 mm | Pending | Reversed connector orientation between ends; spare | 1 |
| 30 | Phillips screw | M1.6 × 4 mm, class 12.9 | Pending | Bag labeled “Phillips Screw M1.6*4, 5pcs” | 5 |
| 31 | Phillips screw | M1.6 × 4 mm | Pending | Bag labeled “Phillips Screw M1.6*4, 8pcs” | 8 |

The UC-01 front case and mount are listed in rows 11–12. Their STEP and STL files are in [step/parts/](step/parts/) and [stl/](stl/). Confirm their fit for your configuration. The complete assembly model is not an additional BOM item.

Rows 22–31 are packed with the kit but are not installed on the arm. Confirm the actual kit contents with your order.

<a id="dimensions"></a>

## Dimensions

[![PiPER-Mate dimension drawing](images/dimensions.png)](drawings/piper-mate.pdf)

Open the [PDF](drawings/piper-mate.pdf) for dimensions or the [DWG](drawings/piper-mate.dwg) for CAD editing. Check revision agreement with the selected model before fabrication.

## Licensing

Hardware licensing remains unresolved. No new license is granted by copying or renaming files.

Found an error in these resources or need help? Contact our [Support Hub](https://fashionstar.com.hk/support/) with your model and the relevant file or item.
