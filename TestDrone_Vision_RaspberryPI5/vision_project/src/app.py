import time
from datetime import datetime
from flask import Flask, render_template, Response, jsonify
import cv2
import sys
import os
import re

# Import project modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from vision_system import VisionSystem
from com_stm32 import ComSTM32
from config import settings

app = Flask(__name__, template_folder='../templates')

# Initialize subsystems
vision = VisionSystem()
communication = ComSTM32(port='COM7', baudrate=115200)

# Global state
drone_state = {
    "mode": "MANUAL",
    "ch1": "--",
    "ch2": "--",
    "ch3": "--",
    "temp": "--",
    "target_acquired": False,
    "last_seen_time": 0,
    "logs": [] # Nouveau : Historique pour le terminal web
}

LOST_TARGET_TIMEOUT = 2.0  # Seconds to wait before dropping AUTO mode

def add_log(message):
    """Ajoute un message horodaté au terminal web"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_line = f"[{timestamp}] {message}"
    drone_state["logs"].append(log_line)
    # Garder seulement les 50 dernières lignes pour ne pas surcharger la mémoire
    if len(drone_state["logs"]) > 50:
        drone_state["logs"].pop(0)

def generate_frames():
    camera = cv2.VideoCapture(0)
    add_log("System initialized. Camera connected.")
    #camera = cv2.VideoCapture(0, cv2.CAP_V4L2) # (ou ton index actuel)

    # OBLIGATOIRE POUR LA CALIBRATION ET POUR FLUIDIFIER L'IMAGE
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # camera.set(cv2.CAP_PROP_FPS, 15) # 15 images/sec pour soulager la Raspberry Pi
    
    while True:
        success, frame = camera.read()
        if not success: break
            
        current_time = time.time()
        
        # 1. PARSE INCOMING DATA (STM32 -> RPi)
        raw_data = communication.read_serial_line()
        if raw_data:
            # --- CASE 1: FLIGHT TELEMETRY (Internal Debug from STM32) ---
            if raw_data.startswith("LOG:"):
                # On retire le préfixe "LOG:" et on ajoute au terminal web
                add_log(raw_data.replace("LOG:", "🛰️ FLIGHT:"))

            # --- CASE 2: AGRICULTURAL SENSORS (Zigbee relay) ---
            elif "CH1=" in raw_data:
                match = re.search(r"CH1=(\d+) CH2=(\d+) CH3=(\d+) Temp_DS1621=([\d.]+)", raw_data)
                if match:
                    new_ch1 = match.group(1)
                    new_ch2 = match.group(2)
                    new_ch3 = match.group(3)
                    new_temp = match.group(4)
                    
                    # On compare avec l'ancien état pour éviter de spammer le terminal
                    if new_ch1 != drone_state["ch1"] or new_temp != drone_state["temp"]:
                        add_log(f"🌱 AGRI: CH1={new_ch1}% | CH2={new_ch2}% | CH3={new_ch3}% | Temp={new_temp}°C")
                    
                    # Mise à jour de l'état global
                    drone_state.update({
                        "ch1": new_ch1, 
                        "ch2": new_ch2, 
                        "ch3": new_ch3, 
                        "temp": new_temp
                    })
        
        # 2. VISION PROCESSING
        marker_info = vision.get_marker_position(frame)
        
        # Gérer l'état de la détection et les logs associés
        if marker_info:
            if not drone_state["target_acquired"]:
                add_log(f"Target Acquired! (ID: {marker_info.get('id', 'Unknown')})")
            drone_state["target_acquired"] = True
            drone_state["last_seen_time"] = current_time
        else:
            if drone_state["target_acquired"]:
                add_log("Warning: Target lost from view.")
            drone_state["target_acquired"] = False

        flight_state = vision.get_flight_state(marker_info, frame.shape)
        
        # 3. SMART SAFETY LOGIC (Hysteresis)
        if drone_state["mode"] == "AUTO":
            time_since_last_seen = current_time - drone_state["last_seen_time"]
            
            if time_since_last_seen > LOST_TARGET_TIMEOUT:
                add_log("AUTO ABORTED: Target lost for > 2s. Switching to MANUAL.")
                drone_state["mode"] = "MANUAL"
                communication.send_command('C')
            elif not marker_info:
                communication.send_command('C')
                flight_state = "RECOVERING SIGNAL..."

        # 4. AUTO EXECUTION
        if drone_state["mode"] == "AUTO" and marker_info:
            communication.send_command(flight_state[0])

        # 5. DRAWING
        display_msg = f"{drone_state['mode']} | {flight_state}"
        if not marker_info and drone_state["mode"] == "AUTO":
            remaining = round(LOST_TARGET_TIMEOUT - (current_time - drone_state["last_seen_time"]), 1)
            display_msg = f"AUTO | SIGNAL LOST (TIMEOUT: {remaining}s)"

        frame = vision.draw_detections(frame, marker_info, flight_state=display_msg)
        
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/mode/<new_mode>', methods=['POST'])
def set_mode(new_mode):
    if new_mode == "AUTO":
        if not drone_state["target_acquired"]:
            return jsonify({"status": "error", "message": "Cannot engage AUTO: No target in sight"})
            
    if drone_state["mode"] != new_mode:
        add_log(f"Pilot switched mode to: {new_mode}")
        drone_state["mode"] = new_mode
        
    return jsonify({"status": "success", "mode": drone_state["mode"]})

@app.route('/api/command/<cmd>', methods=['POST'])
def manual_command(cmd):
    # MANUAL OVERRIDE: Immediate Lock
    if drone_state["mode"] == "AUTO":
        add_log("MANUAL OVERRIDE triggered! Switched to MANUAL.")
        drone_state["mode"] = "MANUAL"
        
    add_log(f"Pilot Command Sent: {cmd}")
    communication.send_command(cmd)
    return jsonify({"status": "success", "mode": "MANUAL"})

@app.route('/api/telemetry')
def get_telemetry():
    return jsonify(drone_state)

@app.route('/api/motors/<action>', methods=['POST'])
def motor_control(action):
    """Handles Arming, Disarming, and Throttle adjustments."""
    commands = {
        "arm": "A",
        "disarm": "D",
        "up": "+",
        "down": "-"
    }
    
    if action in commands:
        cmd = commands[action]
        communication.send_command(cmd)
        add_log(f"Motor Command: {action.upper()} (Sent '{cmd}')")
        return jsonify({"status": "success", "action": action})
    
    return jsonify({"status": "error", "message": "Invalid action"})

if __name__ == '__main__':
    app.run(host=settings.DASHBOARD_HOST, port=settings.DASHBOARD_PORT, debug=False)