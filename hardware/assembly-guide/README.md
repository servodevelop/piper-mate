# Assembly and printing preparation

[← Hardware](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

**On this page:**

- [📂 Select matching files](#select)
- [🖨️ Inspect before printing](#print)
- [🔩 Check assembly and wiring](#assemble)
- [🔌 Continue to first connection](#connect)

<a id="select"></a>

## 📂 Select matching files

Use the [hardware resource table](../README.md#resources) to choose CAD and print files. Open the complete STEP assembly first, then compare the individual parts and UC-01 enclosure. The two print profiles differ; neither has a new physical validation report.

<a id="print"></a>

## 🖨️ Inspect before printing

The projects target Bambu Lab P1S / PLA / 0.4 mm nozzle. Review plate contents, part quantity, orientation, supports, per-object overrides and build volume in your slicer. The native CAD ZIP preserves filenames; resolve any missing assembly references before exporting new geometry. No fit, strength or load rating is inferred from slicer settings.

<a id="assemble"></a>

## 🔩 Check assembly and wiring

Use the [BOM](../README.md#bill-of-materials) for quantities and connector orientations, and the [dimension drawing](../drawings/piper-mate.pdf) for reference. Confirm servo IDs 0–6 and physical zero positions before fitting links. Keep power off when connecting wiring; verify polarity and cable routing before applying power. A complete numbered assembly guide, fastening torques and wiring pinout have not yet been supplied.

<a id="connect"></a>

## 🔌 Continue to first connection

After mechanical and electrical checks, follow [Get started](../../docs/README.md). Verify the leader readings before enabling follower motion.
