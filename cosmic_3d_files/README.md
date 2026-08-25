# 🚀 Cosmic Polar 400 — Modular Firmware Guide

Welcome to the **Cosmic Polar 400** 3D printer firmware documentation! 

This guide explains how the printer works, how the firmware is organized into clean, understandable modules, and how you can build, flash, and test the machine.

---

## 💡 What is the Cosmic Polar 400?

Traditional 3D printers move the print head left/right ($X$) and front/back ($Y$) on a square grid (Cartesian). 

The **Cosmic Polar 400** uses a **Polar Motion Architecture** on a circular build plate:
1. **$R$ Axis (Rotating Build Plate)**: A DC motor rotates the circular build plate ($401\,\text{mm}$ diameter). It uses an **AS5600 magnetic encoder** to measure continuous rotation with high precision.
2. **$\Theta$ Axis (Radial Arm)**: An **MG945 Servo motor** moves the print head in and out along the radius ($0\text{--}200\,\text{mm}$).
3. **$Z$ Axis (Vertical Height)**: A DC motor lifts the gantry vertically ($100\,\text{counts/mm}$).
4. **$E$ Axis (Extruder)**: Controls filament extrusion.

---

## 📁 Codebase Directory Structure

The original 1,400+ line monolithic file has been broken down into self-contained, easy-to-read modules:

```
cosmic3d_poler/
├── Config.h                  # ⚙️ All settings, GPIO pins, and machine constants
├── PolarKinematics.h/.cpp    # 📐 Math engine: Converts (X,Y,Z) to (R, Theta, Z) targets
├── EncoderManager.h/.cpp     # 🧲 TCA9548A multiplexer & AS5600 magnetic encoders
├── MotorControl.h/.cpp       # ⚡ Motor PWM drivers, PID feedback loop & MG945 Servo
├── ThermalControl.h/.cpp     # 🔥 Thermistor reading & hotend/bed heater control
├── GCodeParser.h/.cpp        # 📜 SD card file streaming & G-code line reader
├── NetworkManager.h/.cpp     # 📡 Wi-Fi, MQTT broker communication & NVS memory
├── PrinterStateMachine.h/.cpp# 🔄 Master state machine (Homing, Printing, Jogging)
├── mk9-as5600pid-implementation.ino # 📌 Original single-file sketch (Preserved)
└── README.md                 # 📖 This documentation file
```

---

## 🧩 Module Overview & Responsibilities

### 1. `Config.h` — System Configuration
- **Purpose**: Central location for all hardware pins, constants, and settings.
- **Key Definitions**:
  - `R_BED_DIAMETER_MM = 401.0f` ($401\,\text{mm}$ bed diameter)
  - `R_MOTOR_PULLEY_DIAMETER_MM = 19.0f` ($19\,\text{mm}$ motor pulley)
  - `R_AS5600_COUNTS_PER_MOTOR_REV = 4096.0f` ($12$-bit resolution)
  - `Z_COUNTS_PER_MM = 100.0f` ($100$ counts per mm)

### 2. `PolarKinematics.h / .cpp` — Inverse Kinematics Math
- **Purpose**: Converts standard Cartesian coordinates $(X, Y, Z)$ into Polar values $(R, \theta, Z)$.
- **Functions**:
  - `calculatePolarIK(...)`: Calculates radial distance $R = \sqrt{X^2 + Y^2}$, bed rotation angle $\text{atan2}(Y, X)$, and vertical height $Z$.
  - `generateCartesianSegments(...)`: Splits long straight lines into small $2\,\text{mm}$ segments for smooth arc movements.

### 3. `EncoderManager.h / .cpp` — AS5600 & TCA9548A Drivers
- **Purpose**: Manages communication with magnetic position sensors.
- **Key Details**:
  - The **TCA9548A** multiplexer routes I2C traffic to 3 separate AS5600 sensors (Channel 0 = $R$, Channel 1 = $Z$, Channel 3 = $E$).
  - Prevents spurious angle jumps and counts continuous motor rotations.

### 4. `MotorControl.h / .cpp` — Motor Driving & PID
- **Purpose**: Drives DC motors with PWM signals and sets the MG945 servo angle.
- **Key Details**:
  - `calculatePID(...)`: Calculates proportional-integral-derivative control signals to keep DC motors precisely on target.
  - `driveMotor(...)`: Controls H-Bridge motor direction and speed ($0\text{--}255$ PWM).

### 5. `ThermalControl.h / .cpp` — Temperature & Heaters
- **Purpose**: Monitors hotend and bed temperatures using NTC thermistors.
- **Safety**: Automatically shuts off heaters if thermistor fails or temperature exceeds max safe limits ($280^\circ\text{C}$ hotend, $130^\circ\text{C}$ bed).

### 6. `GCodeParser.h / .cpp` — File Streaming
- **Purpose**: Opens G-code files stored on the MicroSD card, counts total lines, and parses $X, Y, Z, E$ values line by line.

### 7. `NetworkManager.h / .cpp` — Wi-Fi & Wireless File Transfers
- **Purpose**: Connects to Wi-Fi and the Mosquitto MQTT broker (`test.mosquitto.org`).
- **Features**: Receives wireless G-code file transfers in chunked blocks, validates MD5 checksums, and saves settings to NVS non-volatile flash memory.

### 8. `PrinterStateMachine.h / .cpp` — Master State Machine
- **Purpose**: Runs the main state machine loop:
  - `STATE_HOMING_SEEK`: Homes $R$ bed sensor, $Z$ limit switch, and $\Theta$ servo.
  - `STATE_MOTION_READ_LINE`: Reads next G-code command.
  - `STATE_MOTION_PID`: Drives motors to target positions until motion tolerance is reached.

---

## 📌 ESP32-S3 Hardware Pin Map

| Function | GPIO Pin | Voltage | Notes |
| :--- | :---: | :---: | :--- |
| **$R$ Motor PWM 1** | GPIO 14 | 3.3 V | H-bridge IN1 |
| **$R$ Motor PWM 2** | GPIO 7 | 3.3 V | H-bridge IN2 |
| **$Z$ Motor PWM 1** | GPIO 15 | 3.3 V | H-bridge IN1 |
| **$Z$ Motor PWM 2** | GPIO 16 | 3.3 V | H-bridge IN2 |
| **$E$ Motor PWM 1** | GPIO 5 | 3.3 V | H-bridge IN1 |
| **$E$ Motor PWM 2** | GPIO 6 | 3.3 V | H-bridge IN2 |
| **$\Theta$ MG945 Servo** | GPIO 18 | 3.3 V | $50\,\text{Hz}$ PWM Signal |
| **I2C SDA (TCA9548A)** | GPIO 8 | 3.3 V | Data Line |
| **I2C SCL (TCA9548A)** | GPIO 9 | 3.3 V | Clock Line |
| **Hotend Heater Gate** | GPIO 21 | 3.3 V | Gate driver input |
| **Bed Heater Gate** | GPIO 47 | 3.3 V | Gate driver input |
| **Hotend Thermistor** | GPIO 2 | 3.3 V | ADC input |
| **Bed Thermistor** | GPIO 1 | 3.3 V | ADC input |
| **$R$ Home Sensor** | GPIO 46 | 3.3 V | Optical / Hall Sensor |
| **$Z$ Limit Switch** | GPIO 43 | 3.3 V | Microswitch (`USB CDC On Boot = Enabled`) |
| **MicroSD SCK** | GPIO 12 | 3.3 V | SPI Clock |
| **MicroSD MISO** | GPIO 13 | 3.3 V | SPI Data Out |
| **MicroSD MOSI** | GPIO 11 | 3.3 V | SPI Data In |
| **MicroSD CS** | GPIO 10 | 3.3 V | SPI Chip Select |

---

## 🛠️ How to Build and Upload

1. Open **Arduino IDE** (version 2.0+ recommended).
2. Select Board: **ESP32S3 Dev Module**.
3. Configure Tools settings:
   - **USB CDC On Boot**: `Enabled` *(Frees GPIO 43 for Z limit switch)*
   - **Flash Size**: `8MB` or `16MB` (depending on module)
   - **Partition Scheme**: `Default 4MB with spiffs`
4. Open `cosmic3d_poler.ino` or `PrinterStateMachine.h`.
5. Click **Verify** (✓) to compile and **Upload** (➔) to flash the ESP32-S3.

---

## 🧪 Serial Command Testing Guide

Open the **Serial Monitor** at **115200 baud** to send manual test commands:

| Command | Action | Example Output |
| :--- | :--- | :--- |
| `HOME` | Runs full homing sequence ($R, Z, \Theta$) | `[HOME] Polar homing sequence complete ✅` |
| `R <deg>` | Rotates bed to angle in degrees | `R 90` $\to$ Rotates bed to $90^\circ$ ($21,612$ counts) |
| `THETA <mm>` | Moves radial arm to position in mm | `THETA 100` $\to$ Moves servo to $90^\circ$ |
| `Z <mm>` | Moves vertical axis to height in mm | `Z 50` $\to$ Moves Z motor to $50\,\text{mm}$ ($5,000$ counts) |
| `POS?` | Prints current diagnostic status of all encoders | Displays AGC, Magnet Status, Angle & Counts |

---

## 🤝 Questions & Support

If you have questions about wiring or PID tuning, consult `cosmic_polar_400_wiring_diagram.md` in the brain artifacts directory. Happy printing! 🖨️✨
