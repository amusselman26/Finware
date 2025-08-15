## Update Checklist
![Update Checklist](docs/UPDATE_CHECKLIST.md)
## Code Style
- **Language**: C++17 for firmware, Python 3.x for tooling.
- **Formatting**: Use `clang-format` with the repo's `.clang-format` file before committing.
- **Naming**:
  - Variables: `snake_case`
  - Functions: `camelCase`
  - Constants/macros: `UPPER_CASE`
  - Classes: `PascalCase`
- Avoid "magic numbers" — define in `config/*.h` or as `constexpr`.

---

## Git Workflow
- **Default branch**: `main` (stable releases only)
- **Development branch**: `develop` (integration of tested features)
- **Feature branches**: `feature/<short-description>`
- **Commit messages**: Follow [Conventional Commits](https://www.conventionalcommits.org/)  
  Examples:  
  - `feat: add LPS22HB driver`  
  - `fix: correct SPI CS pin for SD card`

---

## Documentation Expectations
- Any **pin changes** → update both `docs/PINMAP.md` and `src/hal/board_pins.h` in the same commit.
- Any **log format changes** → update `docs/LOGFORMAT.md` and the log decoder tool.
- All new modules → add a **minimal example** in `/examples`.

---

## Testing Before Merge
- Pass all hardware bring-up tests relevant to the modified code.
- For I²C devices: verify with the scanner example before integration.
- For SPI devices: verify standalone before shared-bus testing.
- Run integrated telemetry + logging for ≥60s without errors before merging to `develop`.

---

## Pull Request Checklist
- [ ] Code builds without warnings on target board.
- [ ] Examples updated or new example added.
- [ ] Relevant docs updated (PINMAP, LOGFORMAT, DEV_GUIDE if workflow changes).
- [ ] Test plan passed and results noted in PR description.
