#!/Users/whakimi/myenv/bin/python
# follow_mode.py
from base_mode import BaseMode
import math
from math import atan2, degrees

class FollowMode(BaseMode):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants
        self.FOLLOW_DISTANCE = 80    # Ideal distance to maintain from target
        self.CLOSE_THRESHOLD = 60    # Distance considered too close
        self.FAR_THRESHOLD = 100     # Distance considered too far
        self.MAX_SPEED = 4.0
        self.TURN_SPEED = 2.0
        
        # State tracking
        self.target_found = False
        self.search_direction = 1  # 1 or -1 for search direction
        self.lost_target_timer = 0
        self.SEARCH_TIME = 5.0  # Time to search in one direction before switching
        
    def initialize_devices(self):
        super().initialize_devices()  # Call parent's initialize_devices first
        
        # Initialize distance sensors
        self.ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            if sensor:
                sensor.enable(self.TIME_STEP)
                self.ps.append(sensor)
                
        # Initialize LEDs for feedback
        self.leds = []
        for i in range(8):
            led = self.robot.getDevice(f'led{i}')
            if led:
                self.leds.append(led)

    def detect_target(self):
        """
        Detect target using proximity sensors
        Returns: (found, distance, angle)
        """
        # Read all sensors
        readings = [sensor.getValue() for sensor in self.ps]
        max_reading = max(readings)
        
        if max_reading > 0:
            # Target found - determine its relative position
            strongest_sensor = readings.index(max_reading)
            
            # Calculate approximate angle (-180 to 180 degrees)
            # Front sensor is 0 degrees
            angle = (strongest_sensor * 45) % 360
            if angle > 180:
                angle -= 360
                
            return True, max_reading, angle
            
        return False, 0, 0

    def update_leds(self, target_found, target_angle=None):
        """Update LEDs based on target detection status"""
        if target_found:
            # Light up LEDs in the direction of the target
            led_index = int((target_angle + 180) / 45) % 8
            for i, led in enumerate(self.leds):
                led.set(1 if i == led_index else 0)
        else:
            # When searching, create a moving light pattern
            time = self.robot.getTime()
            led_index = int(time * 2) % len(self.leds)
            for i, led in enumerate(self.leds):
                led.set(1 if i == led_index else 0)

    def search_behavior(self):
        """Execute search pattern when target is lost"""
        current_time = self.robot.getTime()
        
        # Switch direction periodically
        if current_time - self.lost_target_timer > self.SEARCH_TIME:
            self.search_direction *= -1
            self.lost_target_timer = current_time
            
        # Spin in place while searching
        self.left_motor.setVelocity(self.TURN_SPEED * self.search_direction)
        self.right_motor.setVelocity(-self.TURN_SPEED * self.search_direction)

    def calculate_follow_speeds(self, distance, angle):
        """Calculate motor speeds for following the target"""
        # Base speeds - moving forward
        left_speed = self.MAX_SPEED
        right_speed = self.MAX_SPEED
        
        # Adjust speeds based on angle to target
        # Convert angle to radians for math
        angle_rad = math.radians(angle)
        
        # Apply angular adjustment
        angle_factor = math.cos(angle_rad)
        turn_amount = math.sin(angle_rad)
        
        # Distance factor - slow down when close, speed up when far
        distance_factor = 1.0
        if distance < self.CLOSE_THRESHOLD:
            # Too close - back up
            distance_factor = -0.5
        elif distance < self.FOLLOW_DISTANCE:
            # Slightly close - slow down
            distance_factor = 0.5
        elif distance > self.FAR_THRESHOLD:
            # Too far - speed up
            distance_factor = 1.5
            
        # Apply factors to base speeds
        base_speed = self.MAX_SPEED * distance_factor * angle_factor
        turn_speed = self.TURN_SPEED * turn_amount
        
        left_speed = base_speed + turn_speed
        right_speed = base_speed - turn_speed
        
        # Ensure speeds are within bounds
        left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))
        
        return left_speed, right_speed

    def update(self):
        """Main update loop for following behavior"""
        # Try to detect target
        target_found, distance, angle = self.detect_target()
        
        # Update LED display
        self.update_leds(target_found, angle if target_found else None)
        
        if target_found:
            # Reset search state
            self.lost_target_timer = self.robot.getTime()
            
            # Calculate and apply motor speeds for following
            left_speed, right_speed = self.calculate_follow_speeds(distance, angle)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            # Debug output
            print(f"Following target - Distance: {distance:.1f}, Angle: {angle:.1f}")
            
        else:
            # Target lost - execute search behavior
            self.search_behavior()
            print("Searching for target...")

    def enter(self):
        """Called when switching to follow mode"""
        print("Entering follow mode!")
        self.target_found = False
        self.lost_target_timer = self.robot.getTime()
        self.search_direction = 1
        
    def exit(self):
        """Called when switching away from follow mode"""
        super().exit()
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)