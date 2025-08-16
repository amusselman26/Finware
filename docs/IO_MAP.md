# I/O Map — Feather STM32F405 Flight Computer (Draft)

**Project:** Feather STM32F405 Flight Computer  
**Doc Owner:** Alexander Musselman  
**Status:** DRAFT (v0.1)  
**Last Updated:** 2025-08-16

> **Purpose:** Single source of truth for connectors, signals, MCU pins, voltages, and notes. Keep this file updated whenever the schematic/PCB or firmware changes. Cross‑reference connector IDs (J1, J2, …) with the schematic/PCB.

---

## 1) Connector Legend & Conventions
- **Connector IDs:** `J#` = external connector, `P#` = power, `TP#` = test pad, `SW#` = switch/jumper.  
- **Pin Order:** For 3‑pin servo‑style connectors use **S–V–G** (Signal, Voltage, Ground).  
- **Voltages:** `VBAT` (LiPo), `5V_SERVO` (BEC/step‑up), `3V3` (logic).  
- **Net naming:** `SERVOx_SIG`, `PYROx_OUT`, `LORA_CS`, `GNSS_TX`, etc.  
- **MCU:** STM32F405 (Feather layout). List both **GPIO** and **Alt Func / Timer** when applicable.  
- **Levels:** All MCU I/O are **3.3 V logic** unless explicitly noted.

> **Change control:** Every pin move must be listed in the changelog at the bottom of this file and reflected in the schematic annotations.

---

## 2) Top‑Level I/O Table

| Conn | Function | Pin / Order | MCU Pin(s) | Voltage | Alt / Timer | Notes |
|---|---|---|---|---|---|---|
| **J1** | **Battery Input** (LiPo) | + / − | — | VBAT (1S or 2S) | — | Connector: **XT30**. Inline fuse or resettable polyfuse recommended. |
| **J2** | **USB‑C (Data+Power)** | CC1, CC2, D+, D−, VBUS, GND | D+/D− → **USB FS** | 5 V in | — | ESD on D+/D−. CC resistors for 5 V sink. VBUS → 5 V rail/buck. |
| **J3** | **SWD Debug** | SWDIO, SWCLK, NRST, 3V3, GND, (SWO opt.) | PA13, PA14, NRST, 3V3, GND, (PB3 SWO) | 3V3 | — | **ARM 10‑pin 1.27 mm** or **STDC14**. Keep near MCU. |
| **J4** | **ARM/Kill Switch** | ARM, GND | GPIO **TBD** | — | — | Jumper or guarded toggle. Gates pyro/servo power enables. |
| **J5** | **I²C Port A (STEMMA/Qwiic)** | SCL, SDA, 3V3, GND | PB10 (SCL), PB11 (SDA) | 3V3 | I²C2 | Use **JST‑SH‑4**. 2x copies allowed for daisy chain. |
| **J6** | **I²C Port B (Aux)** | SCL, SDA, 3V3, GND | PB8 (SCL), PB9 (SDA) | 3V3 | I²C1 | Optional second bus; populate if needed. |
| **J7** | **UART Debug** | GND, TX, RX | PA9 (TX), PA10 (RX) | 3V3 | USART1 | 3‑pin 2.54 mm header for USB‑UART dongle. |
| **J8** | **LoRa Antenna** | RF | — | RF | 50 Ω | **SMA female (edge‑mount)** recommended. Keep 50 Ω launch, ground fence, π‑match pads. |
| **J9** | **GNSS Antenna** | RF | — | RF | 50 Ω | **u.FL** on‑board → pigtail to bulkhead SMA. Add ESD + bias‑tee if using active antenna. |
| **J10** | **Servo 1** | S, V, G | **TBD** | 5–6 V | TIMx_CHy | Use locking housing: **Molex KK/SL** or **JST‑GH‑3**. Decouple 5–6 V rail heavily. |
| **J11** | **Servo 2** | S, V, G | **TBD** | 5–6 V | TIMx_CHy | Same as J13. |
| **J12** | **Servo 3** | S, V, G | **TBD** | 5–6 V | TIMx_CHy | — |
| **J13** | **Servo 4** | S, V, G | **TBD** | 5–6 V | TIMx_CHy | — |
| **J14** | **Pyro A** | +, − | Gate=**TBD** (FET), Sense=**TBD** | VBAT | — | **Pluggable terminal** (3.5–5.08 mm) or **JST‑VH**. Include continuity sense network & TVS. |
| **J15** | **Pyro B** | +, − | Gate=**TBD**, Sense=**TBD** | VBAT | — | Same as Pyro A. |
| **TP1..** | **Test Pads** | 3V3, 5V, GND, SDIO lines, RF feed, pyro rails | — | — | — | Add clearly for probe clips and manufacturing test. |

---

## 8) Changelog
- **v0.1 (2025‑08‑16):** Initial draft with proposed assignments, dual I²C, UART1 for debug/GNSS, four servo placeholders.
