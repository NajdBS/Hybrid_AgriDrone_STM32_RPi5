# ==========================================
# Vision System Configuration
# ==========================================

# ArUco Marker Settings
ARUCO_DICT = "DICT_4X4_50"

# Define your mission zones IDs here
TAKEOFF_MARKER_ID = 0
ZONE_A_MARKER_ID = 1
ZONE_B_MARKER_ID = 2


# CURRENT TARGET: The ID the vision system is currently looking for
MARKER_ZONES = {
    0: "TAKEOFF ZONE",
    1: "ZONE A",
    2: "ZONE B"
}
#MARKER_ID = ZONE_A_MARKER_ID 

# IMPORTANT: Marker size in meters
# Example:  5 cm (measured on your phone screen)
# Example:  16 cm (measured on A4 Sheet)
MARKER_SIZE = 0.17  

# Camera Resolution
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Generic Camera Calibration (for PC webcam testing)
CAMERA_MATRIX = [
    [530.0, 0.0, 320.0],
    [0.0, 530.0, 240.0],
    [0.0, 0.0, 1.0]
]
DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]

# Centering Logic
CENTER_TOLERANCE = 50  # Tolerance in pixels to consider the drone "centered"

# System Logs & Web Dashboard
LOG_DIRECTORY = "./logs"
LOG_LEVEL = "INFO"
DASHBOARD_HOST = "0.0.0.0"
DASHBOARD_PORT = 5000
DASHBOARD_DEBUG = True