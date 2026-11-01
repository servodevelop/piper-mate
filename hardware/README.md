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
- [📋 Bill of Materials](#bill-of-materials)
- [🔢 Joint and servo ID mapping](../docs/specifications.md#joint-ids-and-names)

<a id="resources"></a>

## Resources

| Directory | Description | Status |
| --- | --- | --- |
| [step/](step/) | [Complete assembly STEP](step/piper-mate.step) | ✅ Complete |
| [3mf/](3mf/) | 3D printing files (3MF) | — Coming soon |
| [drawings/](drawings/) | Dimension drawing in PDF and DWG | ✅ Complete |

The STEP assembly is a neutral CAD reference based on the supplied 2026-10-27 release folder. Geometry is unchanged; only the PRODUCT names were normalized to the identifiers below.

The assembly tree groups the printed parts under `3D Printed Parts (11)` and the seven servos under `Servos (7)`. Servo instances are labeled `J1-ID0-RP6-U15H-M` through `J7-ID6-RP6-U15H-M (Handle)`; printed-part labels are `piper-mate-base`, `piper-mate-link 1`–`5`, `piper-mate-handle`, and `piper-mate-finger-ring-right` / `left`. The UC-01 adapter board cover and base are also in this group.

<a id="bill-of-materials"></a>

## Bill of Materials

The table lists **21 installed items**, with servos listed first. Printed-part identifiers match the names shown in the STEP assembly tree exactly, for example `piper-mate-link 1`.

3MF printing files will be provided in [3mf/](3mf/); they are not yet available. Print settings are not listed here; choose layer height, wall count and infill for your printer and filament.

Screw images in rows 17–19 are AI-generated illustrations. The proportions in rows 17–18 have been adjusted for M1.6 × 4 mm; they are not dimensioned manufacturing drawings. Use the specification column for purchasing.

Cable images in rows 13–16 are AI-generated illustrations based on the connector reference. They are not to scale and do not define pin assignments; use the specification column for cable lengths and verify wiring against the actual hardware.

| No. | Part Name | Model / Specification / Part Identifier | Image | Description | Quantity |
| :---: | --- | --- | :---: | --- | :---: |
| 1 | UART bus servo | RP6-U15H-M | <a href="images/rp6-u15h-m.png"><img src="images/rp6-u15h-m.png" alt="RP6-U15H-M" width="90"></a> | Coreless motor; 12-bit absolute encoder; six arm joints plus trigger; J1-ID0–J7-ID6 | 7 |
| 2 | Base | `piper-mate-base` | <a href="images/base.png"><img src="images/base.png" alt="base" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 3 | Link 1 | `piper-mate-link 1` | <a href="images/link1.png"><img src="images/link1.png" alt="link1" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 4 | Link 2 | `piper-mate-link 2` | <a href="images/link2.png"><img src="images/link2.png" alt="link2" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 5 | Link 3 | `piper-mate-link 3` | <a href="images/link3.png"><img src="images/link3.png" alt="link3" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 6 | Link 4 | `piper-mate-link 4` | <a href="images/link4.png"><img src="images/link4.png" alt="link4" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 7 | Link 5 | `piper-mate-link 5` | <a href="images/link5.png"><img src="images/link5.png" alt="link5" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 8 | Handle | `piper-mate-handle` | <a href="images/trigger.png"><img src="images/trigger.png" alt="handle" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 9 | Right finger ring | `piper-mate-finger-ring-right` | <a href="images/finger-ring-1.png"><img src="images/finger-ring-1.png" alt="right finger ring" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 10 | Left finger ring | `piper-mate-finger-ring-left` | <a href="images/finger-ring-2.png"><img src="images/finger-ring-2.png" alt="left finger ring" width="90"></a> | Ivory, PLA 3D printed part | 1 |
| 11 | Adapter board cover | `piper-mate-uc01-front-case` | <a href="images/uc01-front-case.png"><img src="images/uc01-front-case.png" alt="UC-01 adapter board cover" width="90"></a> | Ivory, PLA 3D printed part; UC-01 adapter board cover | 1 |
| 12 | Adapter board base | `piper-mate-uc01-mount` | <a href="images/uc01-mount.png"><img src="images/uc01-mount.png" alt="UC-01 adapter board base" width="90"></a> | Ivory, PLA 3D printed part; UC-01 adapter board base | 1 |
| 13 | Servo cable | A2007H03 × 2<br>100 mm | <a href="images/cables/a2007h03-servo-cable-100mm.png"><img src="images/cables/a2007h03-servo-cable-100mm.png" alt="a2007h03 servo cable 100mm — AI-generated illustration" width="90"></a> | Reversed connector orientation between ends | 4 |
| 14 | Servo cable | A2007H03 × 2<br>140 mm | <a href="images/cables/a2007h03-servo-cable-140mm.png"><img src="images/cables/a2007h03-servo-cable-140mm.png" alt="a2007h03 servo cable 140mm — AI-generated illustration" width="90"></a> | Reversed connector orientation between ends | 1 |
| 15 | Servo cable | A2007H03 × 2<br>250 mm | <a href="images/cables/a2007h03-servo-cable-250mm.png"><img src="images/cables/a2007h03-servo-cable-250mm.png" alt="a2007h03 servo cable 250mm — AI-generated illustration" width="90"></a> | Reversed connector orientation between ends | 1 |
| 16 | Adapter cable | A2007H03 to PH2.0<br>100 mm | <a href="images/cables/a2007h03-ph2-0-adapter-cable-100mm.png"><img src="images/cables/a2007h03-ph2-0-adapter-cable-100mm.png" alt="a2007h03 ph2 0 adapter cable 100mm — AI-generated illustration" width="90"></a> | Same connector orientation at both ends | 1 |
| 17 | Hexagon socket head cap screw | PM M1.6 × 4 mm | <a href="images/fasteners/m1-6-4-socket-head-cap-screw.png"><img src="images/fasteners/m1-6-4-socket-head-cap-screw.png" alt="PM M1.6 × 4 mm hexagon socket head cap screw — AI-generated illustration" width="90"></a> | Black finish, property class 12.9 | 8 |
| 18 | Phillips pan head machine screw | PM M1.6 × 4 mm | <a href="images/fasteners/m1-6-4-phillips-pan-head-screw.png"><img src="images/fasteners/m1-6-4-phillips-pan-head-screw.png" alt="PM M1.6 × 4 mm Phillips pan head machine screw — AI-generated illustration" width="90"></a> | Black finish | 54 |
| 19 | Phillips countersunk head machine screw | PM M3 × 10 mm | <a href="images/fasteners/m3-10-phillips-countersunk-head-screw.png"><img src="images/fasteners/m3-10-phillips-countersunk-head-screw.png" alt="Black M3 × 10 Phillips countersunk head screw — AI-generated illustration" width="90"></a> | Black finish; AI-generated illustration | 1 |
| 20 | Locknut | M3 | <a href="images/fasteners/m3-nut.png"><img src="images/fasteners/m3-nut.png" alt="M3 locknut" width="90"></a> | Black finish | 1 |
| 21 | TTL/USB servo adapter | UC-01 | <a href="images/uc-01.webp"><img src="images/uc-01.webp" alt="UC-01" width="90"></a> | PCBA with DC5521 female power lead; 0.75 mm² lead, 150 mm long | 1 |

## Licensing

Hardware licensing remains unresolved. No new license is granted by copying or renaming files.

Found an error in these resources or need help? Contact our [Support Hub](https://fashionstar.com.hk/support/) with your model and the relevant file or item.
