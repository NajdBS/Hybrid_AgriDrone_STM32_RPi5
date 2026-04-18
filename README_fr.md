# 🚁 Hybrid_AgriUAV_STM32_RPi5

> **Note :** 🇬🇧 [Read the English documentation](README_en.md)

Un drone agricole hybride à navigation assistée, conçu pour la collecte de données au sol et l'asservissement par vision. Ce projet combine le contrôle temps réel strict (STM32) et le traitement lourd haut niveau (Raspberry Pi 5).

## 🧠 Architecture du Système

Le projet est divisé en trois sous-systèmes principaux :

1. **Contrôleur de Vol (STM32F4/L4) - `STM32_FlightController/`**
   - **Moteurs :** Génération PWM (50Hz) via Timers matériels. Configuration "Props Out" (Quad-X).
   - **Capteur :** IMU MPU6050 (I2C) avec filtres passe-bas et Filtre de Kalman (Roll/Pitch).
   - **Asservissement :** Boucle PID pour la stabilisation.
   - **Sécurité :** Kill-switch matériel et logiciel (coupure à >45° d'inclinaison).

2. **Ordinateur de Bord (Raspberry Pi 5) - `RPi_CompanionComputer/`**
   - **Vision :** Caméra RPi, détection de marqueurs ArUco via OpenCV pour le maintien de position.
   - **Interface GCS :** Serveur Web Flask diffusant un flux vidéo compressé (MJPEG).
   - **Contrôle :** Traduction des vecteurs de position en commandes textuelles envoyées au STM32 via UART.

3. **Balise au Sol - `STM32_GroundBeacon/`**
   - **Capteurs :** Humidité du sol (Analogique) et Température (I2C).
   - **Télémétrie :** Transmission des données au drone par module radio (Zigbee ou BLE) via UART.

---

## 🛤️ Méthodologie et Processus de Développement

Ce projet a été développé de manière incrémentale :

### Étape 1 : Le Cerveau Bas Niveau (STM32)
1. **Validation Moteurs :** Création d'un code C simple (Timers HAL) pour générer un signal PWM à 50Hz.
2. **Intégration IMU :** Lecture du MPU6050, application des filtres et extraction des angles.
3. **Test Intermédiaire (UART/USB) :** Avant de connecter la Raspberry Pi, la boucle PID et la commande des moteurs ont été testées depuis un PC via un terminal série (ex: HTerm). Cela passait par le câble USB de la Nucleo (assignation des broches PA2/PA15 pour l'UART2). *(Note : Pour le montage final, l'UART2 a été re-routé sur les broches physiques pour communiquer avec la RPi).*

### Étape 2 : Le Cerveau Haut Niveau (PC puis RPi 5)
1. **Prototypage sur PC :** Le code Python a d'abord été développé sur VS Code (Windows/Mac) avec une webcam classique.
2. **Préparation RPi 5 :** Installation de l'OS et test matériel de la caméra pour s'assurer du bon fonctionnement de la nappe avant l'intégration.

### Étape 3 : Télémétrie Agricole (Configuration & Tests Zigbee)
La communication entre la balise au sol et le drone repose sur une liaison UART. Alors que le BLE ne nécessite pas de configuration complexe, ce projet utilise des modules Zigbee (XBee) pour une meilleure portée.
1. **Configuration Zigbee (Obligatoire) :** Les modules doivent être paramétrés via le logiciel Digi **XCTU**. Ils peuvent être connectés au PC via des adaptateurs **USB-TTL (FTDI)** pour cette étape.
   - **Module Drone :** Mode Coordinator (CE=1).
   - **Module Balise :** Mode Router/End Device (CE=0).
   - **Paramètres cruciaux :** Même réseau (`PAN ID = 1234`) et Baudrate identique au STM32 (`BD = 115200`).
2. **Tests Indépendants :** Une fois configurés, les modules sont testés avec un terminal pour vérifier la transmission sans fil des trames simulées (ex: `CH1=45 Temp=24.50`).

---

## 🧩 Intégration Matérielle : Shield STM32 Custom

Pour garantir une stabilité de vol optimale, le capteur inertiel ne pouvait pas être simplement posé sur une breadboard ou accroché avec des fils volants. *(Voir l'image `nucleo_sensor_shield.jpg`)*

Nous avons conçu un **Shield (carte d'accueil) sur-mesure** pour la carte STM32 Nucleo. Ce circuit imprimé remplit deux rôles critiques :
1. **Fixation mécanique du MPU6050 :** Il offre une interface I2C rigide et parfaitement alignée avec le centre de gravité du drone, limitant ainsi les perturbations et les vibrations parasites.
2. **Routage des communications :** Il centralise les connexions vers les différents modules (I2C pour l'IMU, UART pour la télémétrie).

> ⚠️ **Note d'ingénierie (Erratum Matériel) :** > Le schéma de ce shield correspond à la version V1.0 du PCB, qui comporte une erreur de mapping sur les broches de l'interface UART2. Lors de l'assemblage final du drone, l'UART2 (utilisé pour la communication avec la Raspberry Pi) a dû être re-routé manuellement vers les bonnes broches physiques de la Nucleo.

## 🛠️ Déploiement et Automatisation

### 💡 Astuces de Travail (Headless & Transferts)
- **Travail avec la RPi :** Utilisez **Raspberry Pi Connect** (ou activez SSH/VNC) pour prendre le contrôle à distance de l'ordinateur de bord depuis votre PC.
- **Embarquement du Code :** Vous pouvez utiliser l'extension **Remote-SSH** de VS Code pour éditer les fichiers directement sur la Pi via Wi-Fi.

### Modifications Cruciales pour l'Embarqué (RPi 5)
1. **Port Série :** Modification du code Python pour utiliser le port matériel `/dev/ttyAMA0` au lieu de `COM3`.
2. **L'astuce de la Caméra (`libcamerify`) :** Pour qu'OpenCV puisse lire le flux vidéo natif sur Pi 5, le script **doit obligatoirement** être lancé avec le wrapper `libcamerify` sans quoi l'accès à la caméra échouera.

### Automatisation du Système
1. **Création du Point d'Accès Wi-Fi (Hotspot) :**
   La RPi 5 crée son propre réseau pour être pilotée en plein champ :
   
   Exemple :
   
   ```bash
   sudo nmcli device wifi hotspot ssid Drone_GCS password drone1234
   sudo nmcli connection modify Hotspot connection.autoconnect yes
3. **Automatisation (Service Systemd) :** Un service système (`drone.service`) gère le lancement automatique du programme au démarrage.
   - Ce service assure la résilience du système : en cas de crash, l'application est relancée automatiquement, garantissant une disponibilité constante du flux vidéo et de la télémétrie dès la mise sous tension de la batterie.

# 👨‍💻 Auteur
**Najd BEN SAAD** *Étudiant en Systèmes Embarqués / Ingénierie*

🔗 [Profil LinkedIn](https://www.linkedin.com/in/najd-bensaad/)  
📧 [najd.bensaad@outlook.com](mailto:najd.bensaad@outlook.com)

