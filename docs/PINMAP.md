
# STM32 Feather Breadboard Pin Map

| MCU Pin Label | Function | Connected Device | Device Pin | Notes |
|---------------|----------|------------------|------------|-------|
| 3V3           | Power    | BNO085           | VIN        | Shared with LPS22 VIN |
| GND           | Ground   | BNO085           | GND        | Shared with LPS22 GND |
| SDA (D20)     | I2C Data | BNO085           | SDA        | Address 0x4A |
| SCL (D21)     | I2C Clock| BNO085           | SCL        |  |
| SDA (D20)     | I2C Data | LPS22            | SDA        | Shared bus |
| SCL (D21)     | I2C Clock| LPS22            | SCL        | Shared bus |
| D5            | SPI CS   | LoRa RFM95W      | CS         | Planned for later |
| D6            | SPI RST  | LoRa RFM95W      | RST        | Planned for later |
| D9            | SPI IRQ  | LoRa RFM95W      | DIO0       | Planned for later |
| MISO (D12)    | SPI MISO | LoRa RFM95W      | MISO       | Shared SPI bus |
| MOSI (D11)    | SPI MOSI | LoRa RFM95W      | MOSI       | Shared SPI bus |
| SCK (D13)     | SPI CLK  | LoRa RFM95W      | SCK        | Shared SPI bus |
