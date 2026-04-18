# 📋 Hardware Bill of Materials (BOM) / Nomenclature Matérielle

> **Note :** 🇫🇷 Scroll down for the French version. / La version française se trouve plus bas.

---

## 🇬🇧 English Version

Here is the complete list of hardware components required to replicate the AgriUAV hybrid project.

### 🚁 1. Flight & Power Subsystem (STM32 Flight Controller)
* **Microcontroller:** 1x STM32 Nucleo L432KC Board (or equivalent).
* **Inertial Sensor:** 1x MPU6050 IMU Module (I2C Accelerometer + Gyroscope) or equivalent.
* **Propulsion:**
  * 4x Brushless Motors.
  * 4x Electronic Speed Controllers (ESC).
  * 4x Propellers (Quad-X Configuration).
* **Power:**
  * 1x Power Distribution Board (PDB).

### 🧠 2. Intelligence Subsystem (Companion Computer)
* **Computer:** 1x Raspberry Pi 5 (4GB or 8GB RAM).
* **Storage:** 1x MicroSD Card (32GB min, Class 10) for Raspberry Pi OS installation.
* **Vision:** 1x Raspberry Pi Camera Module v2 (with ribbon cable).

### 🌱 3. Ground Beacon Subsystem
* **Microcontroller:** 1x STM32F407VG Board (or equivalent).
* **Moisture Sensor:** 1x Analog Soil Moisture Sensor.
* **Temperature Sensor:** 1x DS1621 Temperature Sensor (I2C) or similar.

### 📡 4. Communication & Telemetry
* **Radio Modules:** 2x Zigbee Modules (XBee) **OR** 2x BLE Modules (Bluetooth Low Energy, e.g., HC-05).
* **Configuration Interfaces:**
  * 1x USB-TTL Cable (FTDI Adapter) for configuration and debugging.
  * 1x Docking Board (XBee Explorer) or breadboard adapter to connect the radio module to a PC.

<br>

---

## 🇫🇷 Version Française

Voici la liste complète des composants matériels nécessaires pour reproduire le projet hybride AgriUAV.

### 🚁 1. Sous-système Vol & Puissance (STM32 Flight Controller)
* **Microcontrôleur :** 1x Carte STM32 Nucleo L432KC (ou équivalent).
* **Capteur inertiel :** 1x Module IMU MPU6050 (Accéléromètre + Gyroscope I2C) ou équivalent.
* **Propulsion :**
  * 4x Moteurs Brushless.
  * 4x Contrôleurs de vitesse (ESC).
  * 4x Hélices (Configuration Quad-X).
* **Alimentation :**
  * 1x Carte de distribution de puissance (PDB).

### 🧠 2. Sous-système Intelligence (Ordinateur de Bord)
* **Calculateur :** 1x Raspberry Pi 5 (4 Go ou 8 Go de RAM).
* **Stockage :** 1x Carte MicroSD (32 Go min, Classe 10) pour l'installation de Raspberry Pi OS.
* **Vision :** 1x Raspberry Pi Camera Module v2 (avec sa nappe de connexion).

### 🌱 3. Sous-système Balise au Sol (Ground Beacon)
* **Microcontrôleur :** 1x Carte STM32F407VG (ou équivalent).
* **Capteur d'humidité :** 1x Capteur d'humidité du sol analogique.
* **Capteur de température :** 1x Capteur de température DS1621 (I2C) ou similaire.

### 📡 4. Communication & Télémétrie
* **Modules Radio :** 2x Modules Zigbee (XBee) **OU** 2x Modules BLE (Bluetooth Low Energy) type HC-05.
* **Interfaces de configuration :**
  * 1x Câble USB-TTL (Adaptateur FTDI) pour la configuration et le débogage.
  * 1x Platine d'accueil (XBee Explorer) ou adaptateur breadboard pour relier le module radio au PC.
