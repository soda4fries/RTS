#!/Users/whakimi/myenv/bin/python

from controller import Robot, Camera, LED, Motor, DistanceSensor, GPS, Compass
import numpy as np
import cv2
import math
import random

class EPuckPetRobot:
    # Constants
    TIME_STEP = 64
    MAX_SPEED = 6.28
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    LED_ON_DURATION = 3000
    DISTANCE_THRESHOLD = 1000
    
    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        
        # Initialize devices
        self.setup_camera()
        self.setup_motors()
        self.setup_sensors()
        self.setup_leds()
        
        # Initialize states
        self.current_behavior = 'idle'
        self.led_timers = {i: 0 for i in range(8)}
        self.dance_start_time = 0
        self.is_dancing = False
        
        # Initialize ArUco detector with corrected API
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
    def setup_camera(self):
        """Initialize and enable the camera"""
        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.TIME_STEP)
        else:
            print("Warning: Camera device not found!")
            
    def setup_motors(self):
        """Initialize wheel motors"""
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
    def setup_sensors(self):
        """Initialize GPS, compass and distance sensors"""
        self.gps = self.robot.getDevice('gps')
        self.compass = self.robot.getDevice('compass')
        
        if self.gps:
            self.gps.enable(self.TIME_STEP)
        if self.compass:
            self.compass.enable(self.TIME_STEP)
            
        self.ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            if sensor:
                sensor.enable(self.TIME_STEP)
                self.ps.append(sensor)
                
    def setup_leds(self):
        """Initialize LEDs"""
        self.leds = []
        for i in range(8):
            led = self.robot.getDevice(f'led{i}')
            if led:
                self.leds.append(led)
                
    def set_motor_speeds(self, left, right):
        """Set motor speeds with safety limits"""
        left = max(min(left, self.MAX_SPEED), -self.MAX_SPEED)
        right = max(min(right, self.MAX_SPEED), -self.MAX_SPEED)
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
        
    def process_camera(self):
        """Process camera image and detect ArUco markers"""
        if self.camera:
            image = self.camera.getImage()
            if image:
                # Convert to numpy array
                img = np.frombuffer(image, np.uint8).reshape((self.CAMERA_HEIGHT, self.CAMERA_WIDTH, 4))
                # Convert from RGBA to BGR
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                
                try:
                    # Detect ArUco markers
                    corners, ids, rejected = self.aruco_detector.detectMarkers(img)
                    
                    # If markers detected, process them
                    if ids is not None and len(ids) > 0:
                        return img, corners, ids
                    return img, None, None
                except Exception as e:
                    print(f"Error detecting markers: {e}")
                    return img, None, None
        return None, None, None
        
    def dance_behavior(self):
        """Execute a simple dance pattern"""
        current_time = self.robot.getTime() * 1000
        if not self.is_dancing:
            self.dance_start_time = current_time
            self.is_dancing = True
            
        dance_duration = current_time - self.dance_start_time
        if dance_duration < 2000:  # Spin for 2 seconds
            self.set_motor_speeds(2.0, -2.0)
        elif dance_duration < 4000:  # Move forward for 2 seconds
            self.set_motor_speeds(2.0, 2.0)
        else:
            self.is_dancing = False
            self.current_behavior = 'idle'
            
    def follow_marker(self, marker_corners):
        """Follow detected ArUco marker"""
        if marker_corners and len(marker_corners) > 0:
            # Get center of marker
            marker = marker_corners[0][0]
            marker_center_x = np.mean(marker[:, 0])
            
            # Calculate error from center of image
            center_error = marker_center_x - (self.CAMERA_WIDTH / 2)
            
            # Simple proportional control
            Kp = 0.01
            base_speed = 2.0
            
            # Calculate motor speeds
            left_speed = base_speed - (Kp * center_error)
            right_speed = base_speed + (Kp * center_error)
            
            self.set_motor_speeds(left_speed, right_speed)
        else:
            self.set_motor_speeds(0, 0)
            
    def led_pattern(self):
        """Update LED pattern based on current behavior"""
        current_time = self.robot.getTime() * 1000
        
        if self.current_behavior == 'dance':
            # Circular pattern during dance
            led_index = int((current_time / 200) % 8)
            for i, led in enumerate(self.leds):
                led.set(1 if i == led_index else 0)
        else:
            # Default blinking pattern
            for i, led in enumerate(self.leds):
                led.set(1 if current_time % 1000 < 500 else 0)
                
    def run(self):
        """Main control loop"""
        print("Starting E-puck pet robot controller")
        
        while self.robot.step(self.TIME_STEP) != -1:
            try:
                # Process camera image and detect markers
                img, marker_corners, marker_ids = self.process_camera()
                
                # Debug output
                if marker_ids is not None:
                    print(f"Detected markers: {marker_ids}")
                
                # Update behavior based on markers
                if marker_ids is not None:
                    if 1 in marker_ids:  # Marker 1 triggers dance
                        self.current_behavior = 'dance'
                        print("Dance behavior triggered")
                    elif 2 in marker_ids:  # Marker 2 triggers following
                        self.current_behavior = 'follow'
                        print("Follow behavior triggered")
                
                # Execute behaviors
                if self.current_behavior == 'dance':
                    self.dance_behavior()
                elif self.current_behavior == 'follow':
                    self.follow_marker(marker_corners)
                else:  # idle behavior
                    self.set_motor_speeds(0, 0)
                
                # Update LEDs
                self.led_pattern()
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                self.set_motor_speeds(0, 0)  # Safety stop

# Create and run the robot controller
if __name__ == "__main__":
    try:
        controller = EPuckPetRobot()
        controller.run()
    except Exception as e:
        print(f"Error initializing robot: {e}")