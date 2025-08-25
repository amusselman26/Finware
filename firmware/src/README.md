## Software Architecture

![Software_Architecture](Finware/docs/Software_Architecture.png)

## Folder Structure
 - App: Contains the main Arduino sketch and application-level logic. Includes state machine, guidance algorithms, mixer for servos, etc.
 - Services: Contains mid-level logic modules that process data and make decisions. Includes estimator, event detector, logger, etc. Consume data through SensorsFacade and do not talk directly to hardware.
 - Drivers: Implements hardware-specific code for each peripheral.
 - Protocol: Defines packet structures, record formats, and shared enums used across logging and telemetry. Single source of truth for how data is packaged.
 - Util: Provides generic helpers such as task scheduler, ring buffers, unit conversions, etc.
 - Platform: Contains board-specific definitions. BoardPins.hpp defines pin assignments, bus definitions, and chip selects. BuildFlags.hpp defines compile-time options such as debug, simulation, or hardware builds.  
