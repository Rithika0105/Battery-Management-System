# Battery Management System (BMS) - STM32F411RE

A firmware project for a Dual-Battery Management System (BMS) developed for the **STM32F411RE Nucleo-64** board using STM32CubeIDE, HAL drivers, and FreeRTOS.

---

## 📌 Project Overview
This project monitors, protects, and telemeters parameters for a dual-battery pack setup (up to 24V), featuring:
- **Individual Cell / Battery Monitoring**: Measures Battery 1 (up to 12V) and total series Pack / Battery 2 (up to 24V).
- **Current Sensing**: Bidirectional current measurement using an **Allegro ACS712-30A** Hall-effect sensor.
- **Digital Filtering**: 64x digital oversampling for ADC readings to eliminate high-frequency noise and switching ripple.
- **Power Telemetry**: Real-time calculation of Voltage, Current, Power (Watts), and State-of-Charge (SoC).
- **UART Telemetry**: Continuous diagnostics and telemetry streaming via USART2 (115200 Baud, 8N1) through the ST-LINK Virtual COM port.
- **Safety & Protection**: Threshold checking for over-voltage, under-voltage, and over-current events.

---

## ⚙️ Hardware Specifications & Pinout

### Target MCU
- **Microcontroller**: STM32F411RET6 (ARM Cortex-M4 @ up to 84 MHz / 16 MHz HSI)
- **Board**: NUCLEO-F411RE

### Pin Mapping
| Pin  | Function    | Description / Peripheral |
| :--- | :---        | :--- |
| **PA0** | ADC1_IN0    | Battery 1 Tap via Divider (100kΩ / 36kΩ) |
| **PA1** | ADC1_IN1    | Total Pack Tap via Divider (223kΩ / 36kΩ) |
| **PA4** | ADC1_IN4    | ACS712 Current Sensor Output (via 3×15kΩ Divider) |
| **PA2** | USART2_TX   | ST-LINK Virtual COM Port TX (115200 Baud, 8N1) |
| **PA3** | USART2_RX   | ST-LINK Virtual COM Port RX |

### Resistor Dividers & Scaling
- **Battery 1 (12V Range)**:
  - $R_1 = 100\text{ k}\Omega$, $R_2 = 36\text{ k}\Omega$
  - Divider Ratio: $\approx 3.7778$
- **Total Pack (24V Range)**:
  - $R_1 = 223\text{ k}\Omega$, $R_2 = 36\text{ k}\Omega$
  - Divider Ratio: $\approx 7.1944$
- **ACS712-30A Sensor**:
  - $3\times 15\text{ k}\Omega$ resistor network (3:1 ratio) to scale 0–5V sensor output into STM32 0–3.3V ADC range.

---

## 📂 Repository Structure
```
├── Core/
│   ├── Inc/               # Application header files (main.h, stm32f4xx_it.h, etc.)
│   ├── Src/               # Application source files (main.c, stm32f4xx_it.c, etc.)
│   └── Startup/           # Vector table & startup code (startup_stm32f411retx.s)
├── Drivers/
│   ├── CMSIS/             # ARM CMSIS core and device headers
│   └── STM32F4xx_HAL_Driver/ # STMicroelectronics HAL Driver library
├── BMS_NEW.ioc            # STM32CubeMX device configuration file
├── STM32F411RETX_FLASH.ld # Linker script for Flash execution
├── STM32F411RETX_RAM.ld   # Linker script for RAM execution
├── .cproject / .project   # STM32CubeIDE Eclipse project configuration
└── README.md              # Project documentation
```

---

## 🚀 Getting Started

### Prerequisites
- **STM32CubeIDE** (v1.12.0 or later recommended)
- **ST-LINK Drivers** (for flashing and debugging)
- Serial Terminal (PuTTY, Tera Term, Serial Studio, or Arduino Serial Monitor)

### Building and Flashing
1. Open **STM32CubeIDE**.
2. Select **File** > **Import** > **General** > **Existing Projects into Workspace**.
3. Browse to this repository root folder and select the project.
4. Click **Build** (`Ctrl + B`).
5. Connect your NUCLEO-F411RE board via USB.
6. Click **Run** (`Ctrl + F11`) or **Debug** (`F11`) to flash the firmware.

### Telemetry Output
Open your serial terminal configured to:
- **Port**: ST-LINK Virtual COM Port
- **Baud Rate**: `115200`
- **Data Bits**: 8, **Parity**: None, **Stop Bits**: 1
