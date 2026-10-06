# Documentation checks

[← PiPER-Mate](../README.md)　|　🌐 **English** / [简体中文](README.zh.md)

<a id="run"></a>

## ✅ Run checks

Use Python 3.10 or newer. Run from the repository root:

```bash
python -m pip install -r scripts/requirements-docs.txt
python scripts/check_docs.py
python scripts/check_readme_sync.py
git diff --check
```

`check_docs.py` checks local Markdown / HTML link and image targets, including explicit section anchors. It does not request remote URLs. `check_readme_sync.py` compares headings, code blocks, images, tables, links and explicit anchors in bilingual README pairs. Translation meaning still requires manual review.
