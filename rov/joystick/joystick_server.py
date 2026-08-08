#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

"""
ROV Control Server
Reads PS3 joystick input and converts to PWM signals for ROV thrusters.
Listens on port 12345 for TCP connections from ROV clients.
"""



import socket
import threading
import time
import json
import pygame
import sys
import os
from typing import Tuple, Optional

# Centralized config import
try:
    from config import (PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US, PWM_DEADBAND_US,
                        JOYSTICK_PORT)
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from config import (PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US, PWM_DEADBAND_US,
                        JOYSTICK_PORT)

# PS3 sag analog dikey ekseni -> dikey iticiler (heave).
# Bazi surucularde sag stick Y ekseni 3, bazilarinda 4 olarak gorunur.
HEAVE_AXIS = 3

class ROVServer:
    def __init__(self, port: int = 12345):
        self.port = port
        self.running = False
        self.clients = []
        self.joystick = None
        
        # PWM constants from central config.py
        self.PWM_MIN = PWM_MIN_US
        self.PWM_MAX = PWM_MAX_US
        self.PWM_NEUTRAL = PWM_NEUTRAL_US
        
        # Joystick deadzone to prevent drift
        self.DEADZONE = 0.1
        
        # Initialize pygame for joystick (same as test mode)
        pygame.init()
        pygame.joystick.init()
        
    def map_value(self, value: float, from_min: float, from_max: float, to_min: float, to_max: float) -> int:
        """Map a value from one range to another"""
        # Clamp input value
        value = max(from_min, min(from_max, value))
        
        # Map to new range
        mapped = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        return int(mapped)
    
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick input to prevent drift"""
        if abs(value) < self.DEADZONE:
            return 0.0
        return value
    
    def calculate_pwm_values(self, forward_back: float, left_right: float) -> Tuple[int, int]:
        """
        Calculate PWM values for left and right thrusters based on joystick input.
        
        Args:
            forward_back: Forward/backward input (-1.0 to 1.0)
            left_right: Left/right input (-1.0 to 1.0)
            
        Returns:
            Tuple of (left_motor_pwm, right_motor_pwm)
        """
        # Apply deadzone
        forward_back = self.apply_deadzone(forward_back)
        left_right = self.apply_deadzone(left_right)
        
        # Differential mix
        left_speed = forward_back + left_right
        right_speed = forward_back - left_right
        
        # Clamp to -1..1
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        def to_pwm(x: float) -> int:
            # Symmetric scaling around PWM_NEUTRAL_US from config.py
            if abs(x) < 1e-5:
                return self.PWM_NEUTRAL
            us = self.PWM_NEUTRAL + x * (self.PWM_MAX - self.PWM_NEUTRAL)
            return int(max(self.PWM_MIN, min(self.PWM_MAX, us)))

        left_pwm = to_pwm(left_speed)
        right_pwm = to_pwm(right_speed)

        # Clamp to valid range
        left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, left_pwm))
        right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, right_pwm))

        return left_pwm, right_pwm
    
    def init_joystick(self) -> bool:
        """Initialize PS3 joystick"""
        # Reinitialize pygame joystick system to ensure fresh detection
        pygame.joystick.quit()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("No joystick found. Please connect a PS3 controller.")
            return False
        
        # Use the first joystick found
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        print(f"Joystick initialized: {self.joystick.get_name()}")
        print(f"Number of axes: {self.joystick.get_numaxes()}")
        print(f"Number of buttons: {self.joystick.get_numbuttons()}")
        
        return True
    
    def read_joystick_input(self) -> Tuple[float, float]:
        """
        Read joystick input and return forward/back and left/right values.
        
        Returns:
            Tuple of (forward_back, left_right) values from -1.0 to 1.0
        """
        if not self.joystick:
            return 0.0, 0.0
        
        # Process pygame events to update joystick state
        pygame.event.pump()
        
        # Check if joystick is still connected
        if not self.joystick.get_init():
            print("Joystick disconnected, attempting to reconnect...")
            if not self.init_joystick():
                return 0.0, 0.0
        
        # PS3 controller axis mapping (may vary by system):
        # Axis 0: Left stick X (left/right)
        # Axis 1: Left stick Y (up/down)
        # Axis 2: Right stick X (left/right)
        # Axis 3: Right stick Y (up/down)
        
        # Use left stick for movement (same as test mode)
        # Note: Y axis is typically inverted (up = -1, down = +1)
        forward_back = -self.joystick.get_axis(1)  # Invert Y axis
        left_right = self.joystick.get_axis(0)     # X axis
        
        return forward_back, left_right
    
    def handle_client(self, client_socket: socket.socket, address: str):
        """Handle individual client connection"""
        print(f"Client connected from {address}")
        
        # Debug counter for periodic output
        debug_counter = 0
        
        try:
            while self.running:
                # Read joystick input
                forward_back, left_right = self.read_joystick_input()
                
                # Calculate PWM values
                left_pwm, right_pwm = self.calculate_pwm_values(forward_back, left_right)
                
                # Debug output every 25 updates (0.5 seconds at 50Hz)
                debug_counter += 1
                if debug_counter >= 25:
                    debug_counter = 0
                    print(f"[{address}] F/B: {forward_back:6.3f}, L/R: {left_right:6.3f} -> "
                          f"Left: {left_pwm:4d}, Right: {right_pwm:4d}")
                
                # Create data packet
                data = {
                    "timestamp": time.time(),
                    "left_motor_pwm": left_pwm,
                    "right_motor_pwm": right_pwm,
                    "forward_back": round(forward_back, 3),
                    "left_right": round(left_right, 3)
                }
                
                # Send data as JSON
                message = json.dumps(data) + "\n"
                try:
                    client_socket.send(message.encode('utf-8'))
                except socket.error:
                    print(f"Client {address} disconnected")
                    break
                
                # Update rate: 50Hz (20ms)
                time.sleep(0.02)
                
        except Exception as e:
            print(f"Error handling client {address}: {e}")
        finally:
            client_socket.close()
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            print(f"Client {address} disconnected")


def main():
    print("ROV Control Server")
    print("==================")
    print("This server reads PS3 joystick input and converts it to PWM signals")
    print("for ROV thruster control. PWM range: 1000-1980 (neutral: 1470)\n")

    # Initialize Pygame once
    pygame.init()
    pygame.joystick.init()

    joystick = None

    def init_joystick():
        nonlocal joystick
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            return False
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick initialized: {joystick.get_name()} (axes: {joystick.get_numaxes()})")
        return True

    if not init_joystick():
        print("No joystick found. Server will continue and wait for connection.")

    # PWM constants from central config.py
    PWM_MIN = PWM_MIN_US
    PWM_MAX = PWM_MAX_US
    PWM_NEUTRAL = PWM_NEUTRAL_US
    DEADZONE = 0.1

    def read_heave(js):
        """Sag stick dikey ekseni -> up_down. Pozitif = YUKSEL, negatif = DAL."""
        if js is None or js.get_numaxes() <= HEAVE_AXIS:
            return 0.0
        return -js.get_axis(HEAVE_AXIS)  # stick yukari itilince +1

    def apply_deadzone(val):
        return 0.0 if abs(val) < DEADZONE else val

    def to_pwm(x: float) -> int:
        if abs(x) < 1e-5:
            return PWM_NEUTRAL
        us = PWM_NEUTRAL + x * (PWM_MAX - PWM_NEUTRAL)
        return int(max(PWM_MIN, min(PWM_MAX, us)))

    # Start TCP server
    srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_sock.bind(('0.0.0.0', JOYSTICK_PORT))
    srv_sock.listen(1)
    srv_sock.settimeout(0.1)
    print(f"TCP server listening on 0.0.0.0:{JOYSTICK_PORT}")

    client_sock = None
    client_addr = None
    last_print_time = 0
    last_joystick_attempt = 0

    try:
        while True:
            # Accept client if none connected
            if client_sock is None:
                try:
                    client_sock, client_addr = srv_sock.accept()
                    print(f"Client connected from {client_addr}")
                    client_sock.setblocking(False)
                except socket.timeout:
                    pass

            # Check joystick
            if joystick is None or not joystick.get_init():
                now = time.time()
                if now - last_joystick_attempt > 1.0:  # Try reconnect every 1s
                    last_joystick_attempt = now
                    if init_joystick():
                        print("Joystick reconnected successfully.")
                    else:
                        joystick = None  # still disconnected
                forward_back = 0.0
                left_right = 0.0
                up_down = 0.0
            else:
                pygame.event.pump()
                forward_back = apply_deadzone(-joystick.get_axis(1))
                left_right = apply_deadzone(joystick.get_axis(0))
                up_down = apply_deadzone(read_heave(joystick))

            # Differential mix
            left_speed = max(-1.0, min(1.0, forward_back + left_right))
            right_speed = max(-1.0, min(1.0, forward_back - left_right))
            left_pwm = max(PWM_MIN, min(PWM_MAX, to_pwm(left_speed)))
            right_pwm = max(PWM_MIN, min(PWM_MAX, to_pwm(right_speed)))

            # Print every 0.5 seconds
            if time.time() - last_print_time > 0.5:
                print(f"F/B: {forward_back:6.3f}, L/R: {left_right:6.3f}, "
                      f"U/D: {up_down:6.3f} -> Left: {left_pwm:4d}, Right: {right_pwm:4d}")
                last_print_time = time.time()

            # Send to client if connected
            if client_sock is not None:
                payload = {
                    "timestamp": time.time(),
                    "left_motor_pwm": left_pwm,
                    "right_motor_pwm": right_pwm,
                    "forward_back": round(forward_back, 3),
                    "left_right": round(left_right, 3),
                    "up_down": round(up_down, 3),   # + = yuksel, - = dal
                }
                msg = json.dumps(payload) + "\n"
                try:
                    client_sock.send(msg.encode('utf-8'))
                except (BlockingIOError, InterruptedError):
                    pass
                except OSError:
                    print(f"Client {client_addr} disconnected")
                    try:
                        client_sock.close()
                    except:
                        pass
                    client_sock, client_addr = None, None

            time.sleep(0.02)  # 50Hz update rate

    except KeyboardInterrupt:
        print("\nExiting joystick server.")

    finally:
        if client_sock:
            try:
                client_sock.close()
            except:
                pass
        srv_sock.close()
        pygame.quit()

if __name__ == "__main__":
    main()