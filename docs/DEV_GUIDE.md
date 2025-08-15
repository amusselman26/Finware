This document describes the repository structure, workflows, and key practices for developing and maintaining the Flight Computer firmware and hardware.

## Repository Structure
- flight-computer/
- ├─ firmware/ # PlatformIO project
- │ ├─ src/ # Source code
- │ │ ├─ app/ # High-level logic
- │ │ ├─ drivers/ # Sensor/radio/storage drivers
- │ │ ├─ hal/ # Pin mapping, hardware abstraction
- │ │ ├─ util/ # Shared utilities
- │ │ └─ config/ # Build-time config
- │ ├─ include/ # Public headers
- │ ├─ examples/ # Bring-up and test sketches
- ├─ hardware/ # Schematics, PCB, BOM
- ├─ docs/ # Project documentation
- ├─ tools/ # Scripts and utilities
- └─ runs/ # Captured test runs and logs

## Build & Flash
- **PlatformIO** is the primary build system.
- Target board: `adafruit_feather_stm32f405_express`
- Example build:
  ```bash
  pio run
  pio run -t upload
  pio device monitor

# Adding New Hardware
1. Create a driver in src/drivers/ with a minimal public API.
2. Add pins to src/hal/board_pins.h and docs/PINMAP.md.
3. Make a one-file example in /examples for bring-up.
4. Update docs/LOGFORMAT.md if data is logged.

# General Rules of Thumb
Keep hardware abstraction (pin handling, Wire/SPI init) in hal/.
Drivers should not hard-code pins; take them via constructor or config.
Always log schema_version and build hash in output.
Binary log format is preferred; use tools/ decoders for analysis.

# Test Plans
Located in docs/TESTPLANS/.
Each module and integrated build has a repeatable test procedure with expected outputs.

# Documentation Maintenance
Update README.md for repo overview.
Update DEV_GUIDE.md when workflows change.
Keep wiring photos in docs/photos/ linked from relevant examples.
