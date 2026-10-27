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
- [🔧 规格](#specifications)
- [📋 物料清单](#bill-of-materials)
- [🖨️ 打印](#printing)
- [📐 尺寸图](#dimensions)

<a id="resources"></a>

## 下载资料

| 目录 | 说明 | 状态 |
| --- | --- | --- |
| [装配指南](assembly-guide/README.zh.md) | 制作前的资料选择与核对；完整分步教程待补 | 🟡 草稿 |
| [step/](step/) | assembly/ 为整机装配；parts/ 为 12 个零件模型 | ✅ 完整 |
| [stl/](stl/) | [下载全部 STL（12 个零件）](stl/downloads/piper-mate-stl-2026-10-27.zip?raw=true) · [浏览单个 STL](stl/) | ✅ 完整 |
| [drawings/](drawings/) | PDF 与 DWG 尺寸图 | ✅ 完整 |
| [cad-source/](cad-source/) | 可编辑的 SolidWorks 源文件包（20 个原生文件） | 📎 参考 |
| [printing/3mf/](printing/3mf/) | 两份 Bambu Studio 打印工程，仅供参照 | 📎 参考 |

SolidWorks 源文件包保留原文件名，便于追溯装配引用。跨软件使用请优先从 STEP 开始；该包尚未验证为完整的 Pack and Go 导出。两份打印工程使用了不同版本的 UC-01 外壳，未试装不要混用。

<a id="specifications"></a>

## 规格

| 规格 | PiPER-Mate 主臂 |
| --- | --- |
| 臂展／自由度 | 400 mm／6 个手臂关节 + 1 个手柄输入 |
| 关节舵机 | RP6-U15H-M × 7 |
| 位置检测 | 12-bit 绝对值编码器 |
| 通信 | UART 总线、UC-01 USB 接口；示例波特率 1,000,000 |
| 电源 | 12 V、3 A、DC5521 |
| 机械臂重量 | 500 g |
| 从臂连接 | AgileX PiPER 经 USB 转 CAN 连接，1 Mbit/s |

商品规格的关节编号从 0 开始，下表把它与软件从 1 开始的关节名称对应起来。机械行程与从臂指令限位用途不同。

| 舵机 ID／关节 | 产品机械行程 | Python／LeRobot 从臂指令限幅 | 方向 |
| --- | --- | --- | --- |
| 0 / joint1 | −154° 至 +154° | −150° 至 +150° | 反向 |
| 1 / joint2 | 0° 至 195° | 0° 至 180° | 正向 |
| 2 / joint3 | −175° 至 0° | −170° 至 0° | 正向 |
| 3 / joint4 | −102° 至 +102° | −100° 至 +100° | 反向 |
| 4 / joint5 | −142° 至 +142° | −70° 至 +70° | 正向 |
| 5 / joint6 | −120° 至 +120° | −120° 至 +120° | 反向 |

依据：[Python 映射](../Python_SDK/piper_pipermate.py)和 [LeRobot 映射](../Lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py)。各软件路径对手柄输入的换算不同，请参阅对应指南。未核对从臂前，不要将软件限位扩大至主臂机械行程。

<a id="bill-of-materials"></a>

## 物料清单

下表列出装配 BOM 中的 **19 个装机物料条目**，舵机置于首行。打印零件名称与 STEP、STL 文件名一致，描述列保留原 BOM 标识。

这里不记录打印参数：请从 [stl/](stl/) 取模型，按自己的打印机切片，因为层高、壁数和填充取决于机器与耗材。

| 序号 | 零件名称 | 型号／规格／零件标识 | 图片 | 说明 | 数量 |
| :---: | --- | --- | :---: | --- | :---: |
| 1 | UART 总线舵机 | RP6-U15H-M | 待补 | 空心杯电机；12-bit 绝对值编码器；六个手臂关节＋手柄输入 | 7 |
| 2 | 底座 | [base](step/parts/base.step) | <a href="images/base.png"><img src="images/base.png" alt="base" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID0` | 1 |
| 3 | 连杆 1 | [link1](step/parts/link1.step) | <a href="images/link1.png"><img src="images/link1.png" alt="link1" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID1` | 1 |
| 4 | 连杆 2 | [link2](step/parts/link2.step) | <a href="images/link2.png"><img src="images/link2.png" alt="link2" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID2` | 1 |
| 5 | 连杆 3 | [link3](step/parts/link3.step) | <a href="images/link3.png"><img src="images/link3.png" alt="link3" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID3` | 1 |
| 6 | 连杆 4 | [link4](step/parts/link4.step) | <a href="images/link4.png"><img src="images/link4.png" alt="link4" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID4` | 1 |
| 7 | 连杆 5 | [link5](step/parts/link5.step) | <a href="images/link5.png"><img src="images/link5.png" alt="link5" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID5` | 1 |
| 8 | 手柄／扳机 | [trigger](step/parts/trigger.step) | <a href="images/trigger.png"><img src="images/trigger.png" alt="trigger" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID6-0` | 1 |
| 9 | 指环 1 | [finger-ring-1](step/parts/finger-ring-1.step) | <a href="images/finger-ring-1.png"><img src="images/finger-ring-1.png" alt="finger-ring-1" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID6-1` | 1 |
| 10 | 指环 2 | [finger-ring-2](step/parts/finger-ring-2.step) | <a href="images/finger-ring-2.png"><img src="images/finger-ring-2.png" alt="finger-ring-2" width="90"></a> | 米白色 PLA 打印件；BOM 标识 `Piper mate_ID6-2` | 1 |
| 11 | 舵机连接线 | A2007H03 × 2<br>100 mm | 待补 | 两端接头方向相反 | 4 |
| 12 | 舵机连接线 | A2007H03 × 2<br>140 mm | 待补 | 两端接头方向相反 | 1 |
| 13 | 舵机连接线 | A2007H03 × 2<br>250 mm | 待补 | 两端接头方向相反 | 1 |
| 14 | 转接线 | A2007H03 转 PH2.0<br>100 mm | 待补 | 两端接头方向相同 | 1 |
| 15 | 内六角圆柱头螺钉 | PM M1.6 × 4 mm | 待补 | 黑色；12.9 级 | 8 |
| 16 | 十字槽盘头螺钉 | PM M1.6 × 4 mm | 待补 | 黑色 | 54 |
| 17 | 十字槽沉头螺钉 | M3 × 10 | 待补 | 黑色 | 1 |
| 18 | 防松螺母 | M3 | 待补 | 黑色 | 1 |
| 19 | TTL／USB 舵机适配器 | UC-01 | 待补 | PCBA，带 DC5521 母头电源线；线径 0.75 mm²，长 150 mm | 1 |

UC-01 盖子、前壳和安装板可在 [step/parts/](step/parts/) 与 [stl/](stl/) 下载，但本 BOM 没有为它们单列数量。请按实际配置确认适配关系和用量。整机装配模型不额外计作一个物料。

原表另列的实用配件包括紫色 222 低强度螺丝胶、1 m USB-A 转 USB-C 数据线和两个 4 寸 F 夹。包装、备用螺丝与线材，以及另列的裸板 UC-01 不叠加到上述装机数量中。实际套件内容请以订单为准。

<a id="printing"></a>

## 打印

保留两份 Bambu Studio 工程供参照。下表数值来自工程元数据，**不是**打印推荐值；请在切片软件中核对打印盘内容、逐对象覆盖参数、支撑和朝向。

| 工程 | 打印机／材料 | 文件内全局配置 | 状态 |
| --- | --- | --- | --- |
| [piper-mate.3mf](printing/3mf/piper-mate.3mf) | Bambu Lab P1S、0.4 mm 喷嘴、PLA | 0.2 mm 层高；30% 填充；5 层墙 | 仓库原有打印配置 |
| [piper-mate-09-01.3mf](printing/3mf/piper-mate-09-01.3mf) | Bambu Lab P1S、0.4 mm 喷嘴、PLA | 0.2 mm 层高；15% 填充；4 层墙 | 源文件标注“9.1 更新”；适配和强度未验证 |

便携的打印入口是 [stl/](stl/)：12 个独立二进制 STL，外加一个包含同样 12 个文件的压缩包；每个散件与包内条目逐字节一致。UC-01 外壳零件作为独立模型提供，打印前请核对所选工程的版本。

<a id="dimensions"></a>

## 尺寸图

[![PiPER-Mate 尺寸图](images/dimensions.png)](drawings/piper-mate.pdf)

打开 [PDF](drawings/piper-mate.pdf) 查看尺寸，或使用 [DWG](drawings/piper-mate.dwg) 进行 CAD 编辑。制作前请核对图纸与所选模型的版本一致性。

## 许可

硬件许可尚未确定。复制或改名文件不构成新的许可授权。

这些资料有错误或需要帮助？请携带型号和相关文件／条目联系[支持中心](https://fashionstar.com.hk/support/)。
