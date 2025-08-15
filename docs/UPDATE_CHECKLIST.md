# Documentation & Code Update Checklist

**Purpose:** Run through this checklist before merging a branch, closing a milestone, or finishing hardware integration work.  
All checkboxes render in GitHub PRs as interactive items.

---

## 1. General
- [ ] Update **README.md** if project purpose, quick start, or repo structure changed.
- [ ] Update **CHANGELOG.md** with a brief summary of changes.
- [ ] Add/update **photos or diagrams** in relevant example folders (`/examples/*/photos/`).

---

## 2. Hardware Changes
- [ ] Update **PINMAP.md** if pin assignments changed.
- [ ] Export updated **schematics** (PDF/PNG) to `/hardware/`.
- [ ] Update **BOM.md** if components were added, removed, or swapped.
- [ ] Add/update **wiring diagrams** for new or changed hardware.
- [ ] Document bring-up instructions in **DEV_GUIDE.md**.

---

## 3. Firmware/Software Changes
- [ ] Update **CONTRIBUTING.md** if coding standards or commit rules changed.
- [ ] Update **DEV_GUIDE.md** if build/deploy process changed.
- [ ] Add/update **example code** in `/examples/` for new features.
- [ ] Update **driver wrapper** code if underlying Adafruit or hardware drivers changed.
- [ ] Add inline code comments for any complex or non-obvious logic.

---

## 4. Testing & Logs
- [ ] Save **test logs** to `/docs/test_logs/` with date, hardware setup, and results.
- [ ] Update **LOGFORMAT.md** if telemetry/log structure changed.
- [ ] Verify **all examples compile and run** after changes.

---

## 5. Version Control
- [ ] Commit all new/updated images and diagrams alongside code.
- [ ] Push all changes to the remote repository.
- [ ] Tag a new release if this is a major milestone.

---

## 6. Devlog Notes (per update)
Keep a quick bullet list in `/docs/devlog.md` so you can look back on your hardware bring-up history.
- **Date:** `YYYY-MM-DD`
- **Summary:** One-liner of what changed.
- **Details:** Key wiring changes, new libraries, observed bugs, fixes.
- **Photos/Links:** Any relevant wiring pictures or oscilloscope screenshots.
- **Next Steps:** Short plan for what to do next.

Example:
```markdown
### 2025-08-15
- Integrated LPS22 over I²C at 0x4A.
- Confirmed readings match lab reference barometer within ±0.2 hPa.
- Issue: I²C auto-detect failed, manual address required.
- Next: Integrate LoRa module over SPI.
