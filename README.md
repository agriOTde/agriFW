# 🌾 AgriFW — Smart Agriculture IoT Firmware for ESP32

AgriFW is an embedded firmware for ESP32-based IoT systems, designed to monitor and control environmental parameters in hydroponic, aeroponic, and smart-farming setups.  
It integrates sensor data acquisition, MQTT-based cloud communication, OTA firmware updates, and Wi-Fi management, all in a modular, scalable architecture built on ESP-IDF.

---

## 🚀 Features

### 🌐 Wi-Fi Connectivity
Automatic reconnection and status tracking.

### ☁️ MQTT Communication
Real-time data publishing and command reception.

### 🔄 OTA Updates
Seamless over-the-air firmware upgrades.

### 🌡️ Sensor Integration
- SHT31 / SHT41 temperature & humidity sensors  
- Soil-moisture and pH analog sensors (ADC-based)

### 💾 Persistent Storage
NVS (Non-Volatile Storage) for saving schedules, thresholds, and configurations.

### ⚙️ Task-Safe Shared Data Layer
Thread-safe data exchange between FreeRTOS tasks.

### 🔌 UART & HTTP Drivers
Modular components for expansion and cloud API integration.

---

## 🧩 Project Structure

```
agriFW-main/
├── include/               # Header files and global declarations
├── lib/                   # External and shared libraries
├── src/                   # Core source files
│   ├── main.cpp           # Main application entry point
│   ├── mqtt_manager.c     # MQTT handling (publish/subscribe)
│   ├── OTA_manager.c      # OTA update functionality
│   ├── wifi_manager.c     # Wi-Fi connection management
│   ├── nvs_manager.c      # Non-volatile storage (NVS) operations
│   ├── sht31.c / sht41.c  # Temperature & humidity sensor drivers
│   ├── soilMoist.cpp      # Soil-moisture monitoring logic
│   ├── httpClient.c       # HTTP client for cloud communication
│   ├── sharedData.c       # Shared data and mutex management
│   └── singleReadAdc.c    # ADC reading utility for analog sensors
├── platformio.ini         # PlatformIO project configuration
├── CMakeLists.txt         # Build configuration (ESP-IDF)
├── README.md              # Project documentation (this file)
└── test/                  # Unit and integration test files
```

---

## ⚙️ Requirements

### Hardware
- ESP32 DevKit (NodeMCU-32S or equivalent)  
- SHT31 / SHT41 temperature & humidity sensors  
- Analog soil-moisture and pH sensors  
- Wi-Fi network access

### Software
- [PlatformIO](https://platformio.org/) (recommended IDE: VS Code)  
- Python 3.x  
- ESP-IDF SDK (automatically handled by PlatformIO)

---

## 🔧 Setup & Build

```bash
git clone https://github.com/<your-username>/agriFW.git
cd agriFW
pio run --target upload
pio device monitor
```

Open in **VS Code + PlatformIO** — dependencies will install automatically.

---

## 🌱 How It Works

- The firmware connects to Wi-Fi and the configured MQTT broker.  
- Sensor readings are collected periodically and published as JSON.  
- Remote commands (e.g., motor schedules, OTA triggers) are received via MQTT topics.  
- Configuration data and calibration values are stored in ESP32’s NVS flash.  
- OTA updates are securely handled to ensure safe upgrades in the field.

---

## 🧠 Example MQTT Payload

```json
{
  "espClientID": "esp32_7C9EBD1234A5",
  "temperature": 25.6,
  "humidity": 60.3,
  "soilMoisture": 512,
  "ph": 6.8,
  "timestamp": "2025-10-05T14:35:00Z"
}
```

---

## 🧰 Future Improvements

- Saving of all crucial parameters in NVS  
- Actual ACK implementation for received commands  
- MQTT TLS implementation for secure communication  
- Sleep and other power-saving routines  
- Secure PKI and credential data storage in EEPROM  
- Backup and data caching in case of lost MQTT connection  
- Add web-based configuration portal  
- Support LoRaWAN / BLE transport options  
- Add real-time clock (RTC) for offline scheduling  
- Improve sensor calibration routines  
- Dockerized backend for MQTT + PostgreSQL data storage

---

## 🤝 Contributing

Contributions and ideas are welcome!

```bash
git checkout -b feature/your-feature-name
# make changes
git commit -m "Add your feature"
git push origin feature/your-feature-name
```

Submit a pull request 🚀

---

## 📜 License

Licensed under the **MIT License** — see the LICENSE file for details.

---

## 👤 Author

**Abdul Haseeb**  
🌐 [GitHub Profile](https://github.com/abdul2k10023/agriDBContainer)  
💌 haseebarain18@gmail.com  

> “Technology is the seed — data is the soil — and innovation is the harvest.” 🌾
