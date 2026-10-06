# Repository instructions

- Work on PiPER-Mate resources and code in this repository. Keep control changes distinct from documentation restructuring and report which behavior was actually tested.
- English is the default entry. Read both versions before changing a README pair; keep section order, images, tables, links, commands and validation status synchronized. Use `README.md` and `README.zh.md`.
- Directory READMEs start with the title, then the direct-parent link (`← Parent`) followed by a separator and language switch. Bold the current language without a self-link. Longer pages use a localized On this page label and one icon-prefixed link per line with explicit anchors. Do not add READMEs merely to decorate data-only directories.
- Use lowercase and hyphens for resource directories. Preserve Python/ROS identifiers and native CAD filenames needed by assembly references.
- Organize around customer tasks. Preserve exact BOM model suffixes, quantities and connector orientation. Do not include internal cost/packing records, backups or arbitrary historical exports. Mark unverified fit, calibration and performance honestly.
- Existing component licenses and attribution must remain intact. The overall software/hardware license is pending; see `LICENSE.md`.
- After documentation edits run `python scripts/check_docs.py`, `python scripts/check_readme_sync.py` and `git diff --check`. Translation equivalence needs manual review.
- Browser operations default to the Codex in-app browser. Use Chrome only when explicitly requested.
