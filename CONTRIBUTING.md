# Contributing to PiPER-Mate

[← PiPER-Mate](README.md) · [简体中文](CONTRIBUTING.zh.md)

## Report a problem

Use [Issues](https://github.com/servodevelop/piper-mate/issues). Include the repository commit, workflow, OS, Python / ROS / LeRobot and SDK versions, follower firmware, servo IDs, wiring and exact reproduction steps. Describe what you observed rather than calling an untested combination supported. Remove credentials and private information from logs.

## Propose a change

Keep changes focused. Preserve third-party attribution and package names. Explain hardware effects, backwards compatibility and how the change was checked. Do not apply a new project-wide license while [license scope](LICENSE.md) remains pending.

Software folders are now `python-sdk/`, `ros2-humble/` and `lerobot/`; update local commands that used `Python_SDK/`, `ROS2_HUMBLE/` or `Lerobot/`. Python import names and ROS package names are unchanged. Root CAN helper filenames remain available. Hardware files are under `hardware/`.

## Documentation and hardware resources

- English is the default. Keep paired `README.md` / `README.zh.md` files complete and synchronized, including commands, images, tables, status and links.
- Start directory READMEs with a title, then the direct-parent link and language row. Use a vertical icon-prefixed contents list with explicit anchors on longer pages.
- Prefer lowercase, hyphenated resource directories. Preserve framework identifiers and native CAD dependency filenames when required.
- Keep BOM model suffixes, installed quantities and connector orientations exact. Separate spare parts and packaging from assembly quantities.
- Include only useful resources; do not publish cost sheets, personal handoff records or backup files. State revision and validation limits. Keep source-to-target import checks outside customer documentation.

## Validate

Use Python 3.10 or newer for the documentation tools.

```bash
python -m pip install -r scripts/requirements-docs.txt
python scripts/check_docs.py
python scripts/check_readme_sync.py
git diff --check
```

Review translation meaning manually. For code changes, test the affected behavior without moving hardware first, then document an attended hardware test if available. A documentation check or syntax check does not validate physical motion, print fit, training quality or emergency-stop behavior.
