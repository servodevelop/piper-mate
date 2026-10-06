# 参与 PiPER-Mate 开发

[← PiPER-Mate](README.zh.md) · [English](CONTRIBUTING.md)

## 反馈问题

通过 [Issues](https://github.com/servodevelop/piper-mate/issues)反馈，提供仓库提交、软件路径、系统、Python／ROS／LeRobot 与 SDK 版本、从臂固件、舵机 ID、接线及准确复现步骤。描述实际观察结果，不把未测试组合称为已支持。日志中移除凭证与隐私信息。

## 提交修改

保持修改范围清晰，保留第三方署名和软件包名称。说明对硬件的影响、向后兼容性及验证方式。[许可范围](LICENSE.md)仍待确认时，不新增覆盖整个项目的许可。

软件目录现为 `python-sdk/`、`ros2-humble/` 和 `lerobot/`，请更新原先使用 `Python_SDK/`、`ROS2_HUMBLE/` 或 `Lerobot/` 的本地命令。Python 导入名称和 ROS 包名不变，根目录 CAN 辅助脚本保留原名。硬件文件位于 `hardware/`。

## 文档与硬件资料

- 英文为默认入口，成对的 `README.md`／`README.zh.md` 须保持完整同步，包括命令、图片、表格、状态和链接。
- 目录 README 先写标题，再写直接上级链接和语言导航。长页面使用带图标的竖排目录及显式锚点。
- 资源目录优先采用小写和连字符，必要时保留框架标识及原生 CAD 依赖文件名。
- 准确保留 BOM 型号后缀、装机数量和接头方向。备用件与包装不得计入装机数量。
- 仅收录有用资源，不发布成本表、个人交接记录或备份文件。明确版本与验证范围；源文件导入核对记录不放入客户文档。

## 验证

文档工具使用 Python 3.10 或更高版本。

```bash
python -m pip install -r scripts/requirements-docs.txt
python scripts/check_docs.py
python scripts/check_readme_sync.py
git diff --check
```

人工核对翻译含义。修改代码时先验证不驱动硬件的相关行为；具备条件后再记录有人值守的实机测试。文档或语法检查不能验证实际运动、打印适配、训练效果或急停行为。
