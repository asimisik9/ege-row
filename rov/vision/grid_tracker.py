import cv2
import numpy as np
import math

class GridTracker:
    def __init__(self):
        pass

    def process(self, frame):
        """
        Process the downward-facing camera frame to detect the pool floor grid.
        Returns the yaw error (-45 to 45 degrees) relative to the grid.
        """
        if frame is None:
            return None, None

        # Convert to Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur to reduce noise (water ripples, etc.)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        # The white lines on blue tiles should have strong contrast
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
        
        # Probabilistic Hough Transform
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        
        debug_frame = frame.copy()
        angles = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate angle in degrees
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                
                # Normalize angle to -90 to +90
                if angle > 90:
                    angle -= 180
                elif angle < -90:
                    angle += 180
                    
                angles.append(angle)
                
                # Draw lines for debugging
                cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
        if not angles:
            return None, debug_frame
            
        # The pool grid consists of orthogonal lines. 
        # Modulo 90 mapping to align all lines to the same reference axis (-45 to 45).
        mod_angles = []
        for a in angles:
            # Map [-90, 90] to [-45, 45]
            val = (a + 45) % 90 - 45
            mod_angles.append(val)
            
        # Robust estimation of the dominant angle (median)
        yaw_error_deg = np.median(mod_angles)
        
        # Draw the dominant angle as an indicator in the center
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        
        # Draw the target line (green)
        rad = math.radians(yaw_error_deg)
        length = 100
        dx = int(length * math.cos(rad))
        dy = int(length * math.sin(rad))
        cv2.line(debug_frame, (cx - dx, cy - dy), (cx + dx, cy + dy), (0, 255, 0), 3)
        
        cv2.putText(debug_frame, f"Grid Yaw Err: {yaw_error_deg:.1f} deg", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
        return yaw_error_deg, debug_frame
