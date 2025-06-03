# BicycleDataLogger

**BicycleDataLogger** is an open-source, plug-and-play Arduino-based system for gathering movement and orientation data on bicycles and other mobile platforms. Designed for scalability and field deployment, it automatically detects connected sensors and starts logging data with minimal setup.

This project is ideal for researchers, engineers, or citizen science initiatives aiming to collect real-world data from users with minimal technical intervention.

## 🎯 Use Cases

- Bicycle dynamics studies
- Vibration and terrain response logging
- Real-world behavioral tracking (e.g., Baby Vibrations Project)
- Portable multi-sensor logging in other mobile scenarios

## 🔧 Features

- Plug-and-play with auto-detected sensors
- Supports two operation modes:
  - **Standard Mode** (BNO055, RTC, Hall sensor)
  - **High-Vibration Mode** (MPU9250/6050 only)
- Two-button interface for control and calibration
- LED feedback system:
  - Blue = waiting for calibration
  - Red = idle but powered
  - Green = actively measuring
  - Green flash = event flag
  - Blue flash = steering yaw reset
- Logs CSV data to SD card with:
  - Timestamps
  - Orientation
  - Speed
  - Steering angle
  - Flags
  - Optional: Temperature or other sensor data
- Main and secondary housings connected via cable for modularity

## ⚙️ Hardware

- Arduino Nano ESP32
- BNO055 IMUs ×2 (for orientation and steering)
- MPU9250/6050 (for high-frequency vibration)
- DS3231 RTC module
- Hall-effect speed sensor
- SD card module (FAT32 recommended)
- LED (RGB)
- 2× Push buttons (start/stop and flag/reset)
- USB power bank
- Bicycle-mountable main + secondary housings

## 🧰 Supported Sensors

Auto-detected:
- BNO055 (2 units)
- MPU6050 or MPU9250
- DS3231 RTC
- SD card module
- Hall sensor (SPI-based)

## 🧩 Expandability

We encourage contributors to expand the BicycleDataLogger! You can:

- Add support for new I²C or SPI sensors
- Create new operational modes based on connected sensor combinations
- Extend the data logging format to include additional fields (e.g., temperature, vibration frequency)

The current structure supports automatic sensor detection for a core set of devices, and it's straightforward to modify the logic for additional use cases.

## 🚦 Modes

| Mode               | Trigger                              | Behavior |
|--------------------|--------------------------------------|----------|
| **Standard Mode**   | BNO055s connected                    | 100 Hz logging of orientation, speed, steering |
| **Vibration Mode**  | Only MPU connected, no BNO055        | 400 Hz logging of acceleration + 100 Hz other data |

## 🎮 User Interface

| Button             | Action                                             |
|--------------------|----------------------------------------------------|
| Main (on housing)  | Toggle start/stop logging                         |
| Secondary (on fork)| Press = flag event<br>Hold 5s = recalibrate yaw   |

## 📂 Data Logging

- Data is saved to `/data.csv` on the SD card
- One large file per deployment (appends between sessions)
- Timestamped rows with data fields based on connected sensors
- Compatible with Excel, Python, R, and analysis pipelines

## 🚀 Getting Started

### 1. Flash Firmware

Upload `src/main.ino` to your Arduino Nano ESP32 using Arduino IDE or PlatformIO.

### 2. Connect Hardware

Wire up supported sensors (see `src/main.ino` for pin definitions). Sensor detection happens at startup.

### 3. Calibrate & Deploy

- Power the logger via USB power bank
- Wait for the blue LED (calibration mode)
- Calibrate (move to reference position)
- Start logging using the button
- Retrieve the SD card after deployment

## 👩‍🔬 For Researchers

You can build multiple units and send them to test users. Data can be collected offline via SD cards and analyzed centrally. No live monitoring is needed once deployed.

## 🛠️ Customization

- Add more sensors by modifying `setupSensors()` and logging logic
- Create new modes or detection rules based on sensors present
- Expand CSV logging format as needed

## 📄 Licenses

This project is licensed under the MIT License. See [LICENSE](./LICENSE) for details.  
Third-party libraries used are listed in [THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md).

## 🙏 Acknowledgments

Thanks to the Arduino, Adafruit, and open-source communities. Special thanks to the Baby Vibrations Project for helping shape the modular sensor concept.

