"""
Vision system for ArUco marker detection and pose estimation.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import logging
from config import settings

logging.basicConfig(level=settings.LOG_LEVEL)
logger = logging.getLogger(__name__)


class VisionSystem:
    """ArUco marker detection and tracking system."""
    
    def __init__(self):
        # Initialize ArUco dictionary
        aruco_dict_type = getattr(cv2.aruco, settings.ARUCO_DICT)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Camera calibration
        self.camera_matrix = np.array(settings.CAMERA_MATRIX, dtype=np.float32)
        self.dist_coeffs = np.array(settings.DIST_COEFFS, dtype=np.float32)
        
        # # Camera calibration (From YAML File) // (FOR RASPI V2 CAMERA) // (Optional)
        # yaml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "raspicamv2-calibration-640x480.yml")
        # cv_file = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        
        # self.camera_matrix = cv_file.getNode("camera_matrix").mat()
        # self.dist_coeffs = cv_file.getNode("distortion_coefficients").mat()
        
        # cv_file.release()

        # Marker properties
        self.marker_size = settings.MARKER_SIZE
        #self.target_marker_id = settings.MARKER_ID
        self.marker_zones = settings.MARKER_ZONES
        
        logger.info(f"Vision system initialized with {settings.ARUCO_DICT}")
        
    def detect_markers(self, frame):
        """
        Detect ArUco markers in frame.
        Returns: corners, ids, rejected
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
        
        return corners, ids, rejected
        
    def estimate_pose(self, corners):
        """
        Estimate pose of marker.
        Returns: rvec, tvec (rotation and translation vectors)
        """
        # Object points for the marker (4 corners in 3D space)
        marker_points = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            marker_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if success:
            return rvec, tvec
        return None, None
        
    def get_marker_position(self, frame):
        """
        Get position of target marker relative to camera.
        Returns: dict with position info or None
        """
        corners, ids, _ = self.detect_markers(frame)
        
        if ids is None:
            return None
            
        # Find target marker
        marker_found = False
        marker_corners = None

        detected_id = None
        zone_name = ""
        
        # for i, marker_id in enumerate(ids):
        #     if marker_id[0] == self.target_marker_id:
        #         marker_found = True
        #         marker_corners = corners[i][0]
        #         break

        for i, marker_id in enumerate(ids):
            if marker_id[0] in self.marker_zones:  # Check if ID is in our list
                marker_found = True
                marker_corners = corners[i][0]
                detected_id = marker_id[0]
                zone_name = self.marker_zones[detected_id]
                break
                
        if not marker_found:
            return None
            
        # Estimate pose
        rvec, tvec = self.estimate_pose(marker_corners)
        
        if rvec is None:
            return None
            
        # Calculate center in image coordinates
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
        
        # Calculate marker area (for distance estimation)
        area = cv2.contourArea(marker_corners)
        
        # Extract position
        x, y, z = tvec.flatten()
        
        # Convert rotation vector to euler angles
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
        
        if sy > 1e-6:
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            yaw = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            
        return {
            'found': True,
            'x': x,  # meters right
            'y': y,  # meters down
            'z': z,  # meters forward (distance)
            'yaw': np.degrees(yaw),
            'center_x': center_x,
            'center_y': center_y,
            'area': area,
            'corners': marker_corners,
            'id': detected_id, #
            'zone_name': zone_name #
        }
        
    def get_landing_offset(self, marker_info, image_shape):
        """
        Calculate offset from image center for landing guidance.
        Returns: (offset_x, offset_y) in pixels
        """
        if marker_info is None or not marker_info['found']:
            return None, None
            
        height, width = image_shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        offset_x = marker_info['center_x'] - center_x
        offset_y = marker_info['center_y'] - center_y
        
        return offset_x, offset_y
        
    def is_centered(self, marker_info, image_shape):
        """Check if marker is centered in frame."""
        offset_x, offset_y = self.get_landing_offset(marker_info, image_shape)
        
        if offset_x is None:
            return False
            
        distance = np.sqrt(offset_x**2 + offset_y**2)
        return distance < settings.CENTER_TOLERANCE
        
    def draw_detections(self, frame, marker_info, flight_state=None):
        """Draw marker detection visualization."""
        output = frame.copy()
        
        if marker_info and marker_info['found']:
            # Draw marker outline
            corners = marker_info['corners'].astype(int)
            cv2.polylines(output, [corners], True, (0, 255, 0), 2)
            
            # Draw center
            center = (marker_info['center_x'], marker_info['center_y'])
            cv2.circle(output, center, 5, (0, 0, 255), -1)
            
            # Draw crosshair at image center
            h, w = frame.shape[:2]
            cv2.line(output, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 0, 0), 2)
            cv2.line(output, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 0, 0), 2)
            
            # Display position info
            text = f"Dist: {marker_info['z']:.2f}m X: {marker_info['x']:.2f}m Y: {marker_info['y']:.2f}m"
            cv2.putText(output, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            yaw_text = f"Yaw: {marker_info['yaw']:.1f} deg"
            cv2.putText(output, yaw_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            centered = self.is_centered(marker_info, frame.shape)
            status = "CENTERED" if centered else "ADJUSTING"
            color = (0, 255, 0) if centered else (0, 165, 255)
            
            # Show alignment status
            cv2.putText(output, status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # Show detected zone name dynamically
            zone_text = f"DETECTED: {marker_info['zone_name']} (ID {marker_info['id']})"
            cv2.putText(output, zone_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)  
            if flight_state:
                cv2.putText(output, f"STM32: {flight_state}", (10, 150), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return output
   
    # !updated
    def get_flight_state(self, marker_info, frame_shape):
        """Calculates the flight command string based on marker position."""
        if not marker_info:
            return "SEARCHING..."
            
        offset_x, offset_y = self.get_landing_offset(marker_info, frame_shape)
        centered = self.is_centered(marker_info, frame_shape)
        tolerance = settings.CENTER_TOLERANCE

        if centered:
            return "C: HOVERING"
        
        # Prioritize the axis with the largest error
        if abs(offset_x) > abs(offset_y):
            if offset_x < -tolerance:
                return "L: LEFT CORRECTION"
            elif offset_x > tolerance:
                return "R: RIGHT CORRECTION"
        else:
            if offset_y < -tolerance:
                return "F: FORWARD CORRECTION" # F for Forward (A en français)
            elif offset_y > tolerance:
                return "B: BACKWARD CORRECTION" # B for Backward (R en français)
                
        return "ADJUSTING..."

def test_vision_system():
    """Test vision system with webcam or test image."""
    vision = VisionSystem()
    
    # Try to open webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        logger.error("Cannot open camera")
        return
        
    logger.info("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Detect marker
        marker_info = vision.get_marker_position(frame)
        
        # Draw visualization
        output = vision.draw_detections(frame, marker_info)
        
        # # Check if centered
        # if marker_info:
        #     centered = vision.is_centered(marker_info, frame.shape)
        #     status = "CENTERED" if centered else "ADJUSTING"
        #     color = (0, 255, 0) if centered else (0, 165, 255)
        #     cv2.putText(output, status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        # Check if centered
        if marker_info:
            centered = vision.is_centered(marker_info, frame.shape)
            status = "CENTERED" if centered else "ADJUSTING"
            color = (0, 255, 0) if centered else (0, 165, 255)
            
            # Show alignment status
            cv2.putText(output, status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # Show detected zone name dynamically
            zone_text = f"DETECTED: {marker_info['zone_name']} (ID {marker_info['id']})"
            cv2.putText(output, zone_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)



        cv2.imshow('ArUco Detection', output)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_vision_system()

