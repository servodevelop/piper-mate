# 文档检查

[← PiPER-Mate](../README.zh.md)　|　🌐 [English](README.md) / **简体中文**

<a id="run"></a>

## ✅ 运行检查

使用 Python 3.10 或更高版本，在仓库根目录执行：

```bash
python -m pip install -r scripts/requirements-docs.txt
python scripts/check_docs.py
python scripts/check_readme_sync.py
git diff --check
```

`check_docs.py` 检查 Markdown／HTML 本地链接、图片目标及显式章节锚点，不请求远程 URL。`check_readme_sync.py` 比较双语 README 的标题结构、代码块、图片、表格、链接和显式锚点。翻译含义仍需人工核对。
