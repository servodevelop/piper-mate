# PiPER-Mate 硬件资料

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

<p align="center">
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-01.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-01.webp" alt="PiPER-Mate view 1" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-02.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-02.webp" alt="PiPER-Mate view 2" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-03.webp" alt="PiPER-Mate view 3" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-04.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-04.webp" alt="PiPER-Mate view 4" width="18%"></a>
<a href="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-05.webp"><img src="https://fashionstar.com.hk/store/wp-content/uploads/2026/08/piper-mate-main-image-05.webp" alt="PiPER-Mate view 5" width="18%"></a>
</p>

**本页导航：**

- [📂 下载资料](#resources)
- [🔧 规格与指令限位](#specifications)
- [📋 装配 BOM](#bill-of-materials)
- [🖨️ 打印项目](#printing)
- [📐 尺寸图](#dimensions)

<a id="resources"></a>

## 📂 下载资料

| 资料 | 文件 | 用途／状态 |
| --- | --- | --- |
| 整机模型 | [piper-mate.step](step/assembly/piper-mate.step) | 通用 CAD 参考；与各零件的版本对应关系尚未验证 |
| 零件模型 | [step/parts/](step/parts/) | 12 个 STEP 文件；BOM 中的零件名称可直接打开模型 |
| 打印项目 | [printing/3mf/](printing/3mf/) | 原发布配置与 09-01 参考配置；差异见下文 |
| 尺寸图 | [PDF](drawings/piper-mate.pdf) · [DWG](drawings/piper-mate.dwg) | 图纸日期 2026-08-14；不包含装配顺序 |
| 可编辑源文件 | [SolidWorks ZIP](cad-source/piper-mate-solidworks.zip) | 20 个原生文件，保留原文件名；未在 SolidWorks 中验证装配依赖 |
| 制作准备 | [装配与打印指南](assembly-guide/README.zh.md) | 制作前的资料选择和核对；完整装配教程待补 |

原生文件包保留不同零件名称，以免破坏可能存在的装配引用。跨软件使用请优先从 STEP 开始；该源文件包尚不是经过验证的 Pack and Go 导出。未收录备份和渲染会话文件。

<a id="specifications"></a>

## 🔧 规格与指令限位

| 规格 | PiPER-Mate 主臂 |
| --- | --- |
| 臂展／自由度 | 400 mm／6 个手臂关节 + 1 个手柄输入 |
| 关节舵机 | RP6-U15H-M × 7（装配 BOM） |
| 位置检测 | 12-bit 绝对值编码器 |
| 通信 | UART 总线、UC-01 USB 接口；示例波特率 1,000,000 |
| 主臂电源 | 12 V、3 A、DC5521 |
| 机械臂重量 | 500 g（商品规格） |
| 从臂连接 | AgileX PiPER 经 USB 转 CAN 连接，1 Mbit/s |

[商品规格](https://fashionstar.com.hk/store/product/robot-arm-piper-mate/)的关节编号从 0 开始，下表与软件从 1 开始的关节名称对应。机械行程与从臂指令限位用途不同。

| 舵机 ID／关节 | 产品机械行程 | Python／LeRobot 从臂指令限幅 | 方向 |
| --- | --- | --- | --- |
| 0 / joint1 | −154° 至 +154° | −150° 至 +150° | 反向 |
| 1 / joint2 | 0° 至 195° | 0° 至 180° | 正向 |
| 2 / joint3 | −175° 至 0° | −170° 至 0° | 正向 |
| 3 / joint4 | −102° 至 +102° | −100° 至 +100° | 反向 |
| 4 / joint5 | −142° 至 +142° | −70° 至 +70° | 正向 |
| 5 / joint6 | −120° 至 +120° | −120° 至 +120° | 反向 |

依据：[Python 映射](../python-sdk/piper_pipermate.py)和 [LeRobot 映射](../lerobot/lerobot_teleoperator_pipermate/lerobot_teleoperator_pipermate/config_starai_pipermate.py)。各软件路径对手柄输入的换算不同，请参阅对应指南。未核对从臂前，不要将软件限位扩大至主臂机械行程。

<a id="bill-of-materials"></a>

## 📋 装配 BOM

以下 **19 个装机物料条目**保留装配 BOM 中的型号、数量和连接线方向，舵机置于首行。打印零件文件名与下表一致，描述列保留原 BOM 标识。

| 序号 | 名称 | 图片 | 规格／描述 | 数量 | 备注 |
| :---: | --- | :---: | --- | :---: | --- |
| 1 | RP6-U15H-M | 待补 | 空心杯 UART 总线舵机 | 7 | 六个手臂关节＋手柄输入 |
| 2 | [base](step/parts/base.step) | 待补 | PLA；Piper mate_ID0 | 1 | 底座 |
| 3 | [link1](step/parts/link1.step) | 待补 | PLA；Piper mate_ID1 | 1 | 连杆 1 |
| 4 | [link2](step/parts/link2.step) | 待补 | PLA；Piper mate_ID2 | 1 | 连杆 2 |
| 5 | [link3](step/parts/link3.step) | 待补 | PLA；Piper mate_ID3 | 1 | 连杆 3 |
| 6 | [link4](step/parts/link4.step) | 待补 | PLA；Piper mate_ID4 | 1 | 连杆 4 |
| 7 | [link5](step/parts/link5.step) | 待补 | PLA；Piper mate_ID5 | 1 | 连杆 5 |
| 8 | [trigger](step/parts/trigger.step) | 待补 | PLA；Piper mate_ID6-0 | 1 | 手柄／扳机 |
| 9 | [finger-ring-1](step/parts/finger-ring-1.step) | 待补 | PLA；Piper mate_ID6-1 | 1 | 指环 1 |
| 10 | [finger-ring-2](step/parts/finger-ring-2.step) | 待补 | PLA；Piper mate_ID6-2 | 1 | 指环 2 |
| 11 | 舵机连接线 | 待补 | A2007H03 × 2；100 mm | 4 | 双头反向 |
| 12 | 舵机连接线 | 待补 | A2007H03 × 2；140 mm | 1 | 双头反向 |
| 13 | 舵机连接线 | 待补 | A2007H03 × 2；250 mm | 1 | 双头反向 |
| 14 | 转接线 | 待补 | A2007H03 转 PH2.0；100 mm | 1 | 双头同向 |
| 15 | 内六角螺丝 | 待补 | PM M1.6 × 4 mm；黑色；12.9 级 | 8 | 装机用 |
| 16 | 十字槽螺丝 | 待补 | PM M1.6 × 4 mm | 54 | 装机用 |
| 17 | 沉头螺丝 | 待补 | M3 × 10；黑色；十字槽 | 1 | 装机用 |
| 18 | 防松螺母 | 待补 | M3；黑色 | 1 | 装机用 |
| 19 | UC-01 | 待补 | PCBA，带 DC5521 母头电源线 | 1 | 线径 0.75 mm²；长度 150 mm |

UC-01 盖子、前壳和安装板可在 [step/parts/](step/parts/)下载，但该 BOM 没有单独列明数量。请按实际配置确认适配关系和用量。整机装配模型不额外计作一个物料。

原表另列的实用配件包括紫色 222 低强度螺丝胶、1 m USB-A 转 USB-C 数据线和两个 4 寸 F 夹。包装、备用螺丝／线材及另列的裸板 UC-01 不叠加到装机数量中。实际套件内容请以订单为准。

<a id="printing"></a>

## 🖨️ 打印项目

| 项目 | 打印机／材料 | 文件内全局配置 | 状态 |
| --- | --- | --- | --- |
| [piper-mate.3mf](printing/3mf/piper-mate.3mf) | Bambu Lab P1S、0.4 mm 喷嘴、PLA | 0.2 mm 层高；30% 填充；5 层墙 | 仓库原有打印配置 |
| [piper-mate-09-01.3mf](printing/3mf/piper-mate-09-01.3mf) | Bambu Lab P1S、0.4 mm 喷嘴、PLA | 0.2 mm 层高；15% 填充；4 层墙 | 源文件标注“9.1 更新”；适配和强度未验证 |

上述参数来自项目元数据，不是打印推荐值。请在切片软件中核对逐对象覆盖参数、支撑和朝向。09-01 项目的 UC-01 外壳布局不同，未经试装不要混用版本。尚未提供独立 STL 文件。另一个以“手柄”命名的源项目实际上包含多个手臂零件，因此不作为仅手柄打印项目提供。

<a id="dimensions"></a>

## 📐 尺寸图

[![PiPER-Mate 尺寸图](images/dimensions.png)](drawings/piper-mate.pdf)

打开 [PDF](drawings/piper-mate.pdf)查看尺寸，或使用 [DWG](drawings/piper-mate.dwg)进行 CAD 编辑。制作前请核对图纸与所选模型的版本一致性。
