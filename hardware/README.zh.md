# PiPER-Mate 硬件资料

[← PiPER-Mate](../README.zh.md)　|　<sub>[English](README.md) / **简体中文**</sub>

<p align="center">
  <a href="images/piper-mate-main-image-01.webp"><img src="images/piper-mate-main-image-01.webp" alt="PiPER-Mate 产品图 1" width="15%"></a>
  <a href="images/piper-mate-main-image-02.webp"><img src="images/piper-mate-main-image-02.webp" alt="PiPER-Mate 产品图 2" width="15%"></a>
  <a href="images/piper-mate-main-image-03.webp"><img src="images/piper-mate-main-image-03.webp" alt="PiPER-Mate 产品图 3" width="15%"></a>
  <a href="images/piper-mate-main-image-04.webp"><img src="images/piper-mate-main-image-04.webp" alt="PiPER-Mate 产品图 4" width="15%"></a>
  <a href="images/piper-mate-main-image-05.webp"><img src="images/piper-mate-main-image-05.webp" alt="PiPER-Mate 产品图 5" width="15%"></a>
</p>

**本页导航：**

- [📂 下载资料](#resources)
- [📋 物料清单](#bill-of-materials)
- [🔢 关节与舵机 ID 对照](../docs/specifications.zh.md#joint-ids-and-names)

<a id="resources"></a>

## 下载资料

| 目录 | 说明 | 状态 |
| --- | --- | --- |
| [step/](step/) | [整机装配 STEP](step/piper-mate.step) | ✅ 完整 |
| [3mf/](3mf/) | 3D 打印文件（3MF） | — 待提供 |
| [drawings/](drawings/) | PDF 与 DWG 尺寸图 | ✅ 完整 |

装配 STEP 为通用 CAD 参考，基于 2026-10-27 发布文件夹。

装配设计树把打印件归入 `3D Printed Parts (11)`，七个舵机归入 `Servos (7)`；舵机名称为 `J1-ID0-RP6-U15H-M` 至 `J7-ID6-RP6-U15H-M (Handle)`。打印件名称为 `piper-mate-base`、`piper-mate-link 1`–`5`、`piper-mate-handle`、`piper-mate-finger-ring-right`／`left`，UC-01 转接板盖板与底座也在此组。

<a id="bill-of-materials"></a>

## 物料清单

BOM 共 **21 项装机物料**，舵机置于首行。打印件的“型号 / 规格”栏使用统一的零件标识。

3MF 打印文件将放在 [3mf/](3mf/)，目前待提供。本表不记录打印参数；层高、壁数和填充率应根据打印机与耗材选择。

第 17–19 项螺钉图片为 AI 生成示意图；第 17、18 项已按 M1.6 × 4 mm 调整外观比例，不作为带尺寸的加工图纸。采购时以规格列为准。

第 13–16 项连接线图片为依据接头参考图制作的 AI 生成示意图，不按比例绘制，也不作为引脚接线依据；线长以规格列为准，接线请核对实际硬件。

| 序号 | 零件名称 | 型号 / 规格 | 图片 | 说明 | 数量 |
| :---: | --- | --- | :---: | --- | :---: |
| 1 | UART 总线舵机 | RP6-U15H-M | <a href="images/rp6-u15h-m.png"><img src="images/rp6-u15h-m.png" alt="RP6-U15H-M" width="90"></a> | 空心杯电机；12-bit 绝对值编码器；六个手臂关节＋手柄输入；J1-ID0–J7-ID6 | 7 |
| 2 | 基座 | piper-mate-base | <a href="images/base.png"><img src="images/base.png" alt="base" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 3 | 连杆 1 | piper-mate-link1 | <a href="images/link1.png"><img src="images/link1.png" alt="link1" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 4 | 连杆 2 | piper-mate-link2 | <a href="images/link2.png"><img src="images/link2.png" alt="link2" width="90"></a> | 黄色 PLA 打印件 | 1 |
| 5 | 连杆 3 | piper-mate-link3 | <a href="images/link3.png"><img src="images/link3.png" alt="link3" width="90"></a> | 黄色 PLA 打印件 | 1 |
| 6 | 连杆 4 | piper-mate-link4 | <a href="images/link4.png"><img src="images/link4.png" alt="link4" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 7 | 连杆 5 | piper-mate-link5 | <a href="images/link5.png"><img src="images/link5.png" alt="link5" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 8 | 手柄 | piper-mate-handle | <a href="images/trigger.png"><img src="images/trigger.png" alt="handle" width="90"></a> | 黄色 PLA 打印件 | 1 |
| 9 | 右指环 | piper-mate-finger-ring-right | <a href="images/finger-ring-1.png"><img src="images/finger-ring-1.png" alt="right finger ring" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 10 | 左指环 | piper-mate-finger-ring-left | <a href="images/finger-ring-2.png"><img src="images/finger-ring-2.png" alt="left finger ring" width="90"></a> | 米白色 PLA 打印件 | 1 |
| 11 | 转接板盖板 | piper-mate-uc01-front-case | <a href="images/uc01-front-case.png"><img src="images/uc01-front-case.png" alt="UC-01 转接板盖板" width="90"></a> | 米白色 PLA 打印件；UC-01 转接板盖板 | 1 |
| 12 | 转接板底座 | piper-mate-uc01-mount | <a href="images/uc01-mount.png"><img src="images/uc01-mount.png" alt="UC-01 转接板底座" width="90"></a> | 黄色 PLA 打印件；UC-01 转接板底座 | 1 |
| 13 | 舵机连接线 | A2007H03 × 2<br>100 mm | <a href="images/cables/a2007h03-servo-cable-100mm.png"><img src="images/cables/a2007h03-servo-cable-100mm.png" alt="舵机连接线——AI 生成示意图" width="90"></a> | 两端接头方向相反 | 4 |
| 14 | 舵机连接线 | A2007H03 × 2<br>140 mm | <a href="images/cables/a2007h03-servo-cable-140mm.png"><img src="images/cables/a2007h03-servo-cable-140mm.png" alt="舵机连接线——AI 生成示意图" width="90"></a> | 两端接头方向相反 | 1 |
| 15 | 舵机连接线 | A2007H03 × 2<br>250 mm | <a href="images/cables/a2007h03-servo-cable-250mm.png"><img src="images/cables/a2007h03-servo-cable-250mm.png" alt="舵机连接线——AI 生成示意图" width="90"></a> | 两端接头方向相反 | 1 |
| 16 | 转接线 | A2007H03 转 PH2.0<br>100 mm | <a href="images/cables/a2007h03-ph2-0-adapter-cable-100mm.png"><img src="images/cables/a2007h03-ph2-0-adapter-cable-100mm.png" alt="转接线——AI 生成示意图" width="90"></a> | 两端接头方向相同 | 1 |
| 17 | 内六角圆柱头螺钉 | PM M1.6 × 4 mm | <a href="images/fasteners/m1-6-4-socket-head-cap-screw.png"><img src="images/fasteners/m1-6-4-socket-head-cap-screw.png" alt="PM M1.6 × 4 mm 内六角圆柱头螺钉——AI 生成示意图" width="90"></a> | 黑色；12.9 级 | 8 |
| 18 | 十字槽盘头螺钉 | PM M1.6 × 4 mm | <a href="images/fasteners/m1-6-4-phillips-pan-head-screw.png"><img src="images/fasteners/m1-6-4-phillips-pan-head-screw.png" alt="PM M1.6 × 4 mm 十字槽盘头螺钉——AI 生成示意图" width="90"></a> | 黑色 | 54 |
| 19 | 十字槽沉头螺钉 | PM M3 × 10 mm | <a href="images/fasteners/m3-10-phillips-countersunk-head-screw.png"><img src="images/fasteners/m3-10-phillips-countersunk-head-screw.png" alt="黑色 M3 × 10 十字槽沉头螺钉——AI 生成示意图" width="90"></a> | 黑色；AI 生成示意图 | 1 |
| 20 | 防松螺母 | M3 | <a href="images/fasteners/m3-nut.png"><img src="images/fasteners/m3-nut.png" alt="M3 防松螺母" width="90"></a> | 黑色 | 1 |
| 21 | TTL／USB 舵机适配器 | UC-01 | <a href="images/uc-01.webp"><img src="images/uc-01.webp" alt="UC-01" width="90"></a> | PCBA，带 DC5521 母头电源线；线径 0.75 mm²，长 150 mm | 1 |

## 许可

硬件许可尚未确定。复制或改名文件不构成新的许可授权。

这些资料有错误或需要帮助？请携带型号和相关文件／条目联系[支持中心](https://fashionstar.com.hk/support/)。
