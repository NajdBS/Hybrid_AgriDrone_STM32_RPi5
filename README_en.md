# 🚁 Hybrid_AgriUAV_STM32_RPi5

> **Note :** 🇫🇷 [Lire la documentation en français](README_fr.md)

An assisted-navigation agricultural drone designed for ground data collection and vision-based position holding. This project combines strict real-time control (STM32) with high-level heavy processing (Raspberry Pi 5).

## 🧠 System Architecture

The project is divided into three main subsystems:

1. **Flight Controller (STM32F4/L4) - `STM32_FlightController/`**
   - **Motors:** PWM generation (50Hz) via hardware Timers. Quad-X "Props Out" configuration.
   - **Sensor:** MPU6050 IMU (I2C) with low-pass filters and Kalman Filter (Roll/Pitch).
   - **Control:** PID loop for stabilization.
   - **Safety:** Hardware and software kill-switch (cut-off at >45° tilt).

2. **Companion Computer (Raspberry Pi 5) - `RPi_CompanionComputer/`**
   - **Vision:** RPi Camera, ArUco marker detection via OpenCV for position holding.
   - **GCS Interface:** Flask Web Server broadcasting a compressed video feed (MJPEG).
   - **Control:** Translates position vectors into text commands sent to the STM32 via UART.

3. **Ground Beacon - `STM32_GroundBeacon/`**
   - **Sensors:** Soil moisture (Analog) and Temperature (I2C).
   - **Telemetry:** Data transmission to the drone via radio modules (Zigbee or BLE) over UART.

---

## 🛤️ Development Process & Methodology

This project followed an incremental development path:

### Step 1: Low-Level Brain (STM32)
1. **Motor Validation:** Developed a simple C code (HAL Timers) to generate 50Hz PWM signals.
2. **IMU Integration:** MPU6050 raw data reading, filtering, and angle extraction.
3. **Intermediate Testing (UART/USB):** Before RPi integration, the PID loop and motor control were tested from a PC using a serial terminal (e.g., HTerm). This used the Nucleo's USB cable (assigning pins PA2/PA15 for UART2). *(Note: For final assembly, UART2 was re-routed to physical pins for RPi communication).*

### Step 2: High-Level Brain (PC to RPi 5)
1. **PC Prototyping:** Python code was first developed in VS Code using a standard webcam.
2. **RPi 5 Setup:** OS installation and camera hardware validation to ensure the ribbon cable worked before integration.

### Step 3: Agricultural Telemetry (Zigbee Configuration & Tests)
Communication between the beacon and the drone uses UART. While BLE requires no complex setup, this project uses Zigbee (XBee) for better range.
1. **Zigbee Configuration (Mandatory):** Modules must be configured via Digi **XCTU**. You can connect them to a PC using **USB-TTL (FTDI)** adapters for this step.
   - **Drone Module:** Coordinator mode (CE=1).
   - **Beacon Module:** Router/End Device mode (CE=0).
   - **Crucial Parameters:** Same `PAN ID = 1234` and Baudrate matching the STM32 (`BD = 115200`).
2. **Independent Testing:** Once configured, modules were tested with a terminal to verify wireless transmission of simulated frames (e.g., `CH1=45 Temp=24.50`).

---

## 🛠️ Deployment & Automation

### 💡 Workflow Tips (Headless & Transfer)
- **Working with RPi:** Use **Raspberry Pi Connect** (or enable SSH/VNC) to remotely control the onboard computer from your PC.
- **Code Deployment:** Use the **Remote-SSH** VS Code extension to edit files directly on the Pi over Wi-Fi.

### Crucial Embedded Adjustments (RPi 5)
1. **Serial Port:** Updated the Python code to use the hardware port `/dev/ttyAMA0` instead of `COM3`.
2. **Camera Wrapper (`libcamerify`):** For OpenCV to access the Pi 5's native camera stream, the script **must** be launched using the `libcamerify` wrapper, otherwise the camera access will fail.

### System Automation
1. **Wi-Fi Hotspot Creation:**
   The RPi 5 generates its own network for field operation:
   
   Example :
   
   ```bash
   sudo nmcli device wifi hotspot ssid Drone_GCS password drone1234
   sudo nmcli connection modify Hotspot connection.autoconnect yes
3. **Automation (Systemd Service):** A system service (`drone.service`) handles the automatic execution of the program at boot time.
   - This service ensures system resilience: in the event of a crash, the application restarts automatically, ensuring continuous availability of the video stream and telemetry as soon as the battery is connected.

# 👨‍💻 Author
**Najd BEN SAAD** *Embedded Systems / Engineering Student*

🔗 [LinkedIn Profile](https://www.linkedin.com/in/najd-bensaad/)  
📧 [najd.bensaad@outlook.com](mailto:najd.bensaad@outlook.com)
