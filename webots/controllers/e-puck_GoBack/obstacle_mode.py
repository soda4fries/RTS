# obstacle_mode.py
from base_mode import BaseMode
import math
import random

class ObstacleMode(BaseMode):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants
        self.DANGER_THRESHOLD = 100    # Very close to obstacle
        self.CAUTION_THRESHOLD = 80    # Getting close to obstacle
        self.SAFE_SPEED = 4.0          # Normal movement speed
        self.CAUTIOUS_SPEED = 2.0      # Speed when near obstacles
        self.TURN_SPEED = 3.0          # Speed for turning
        
        # State tracking
        self.stuck_timer = 0
        self.last_position = None
        self.stuck_threshold = 2.0  # Time to consider robot as stuck
        self.min_progress = 0.01    # Minimum distance to consider as movement
        
    def initialize_devices(self):
        super().initialize_devices()  # Call parent's initialize_devices first
        
        # Initialize GPS for position tracking
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.TIME_STEP)
        
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

    def get_sensor_readings(self):
        """Get readings from all proximity sensors"""
        readings = []
        for sensor in self.ps:
            readings.append(sensor.getValue())
        return readings

    def update_leds(self, sensor_readings):
        """Update LEDs based on sensor readings"""
        for i, (reading, led) in enumerate(zip(sensor_readings, self.leds)):
            if reading > self.DANGER_THRESHOLD:
                led.set(1)  # Red for danger
            elif reading > self.CAUTION_THRESHOLD:
                # Blink for caution
                time = self.robot.getTime()
                led.set(1 if int(time * 2) % 2 else 0)
            else:
                led.set(0)  # Off for safe

    def check_if_stuck(self):
        """Check if the robot is stuck"""
        current_pos = self.gps.getValues()
        current_time = self.robot.getTime()
        
        if self.last_position is None:
            self.last_position = current_pos
            self.stuck_timer = current_time
            return False
            
        # Calculate distance moved
        distance = math.sqrt(sum((c - l)**2 for c, l in zip(current_pos, self.last_position)))
        
        if distance < self.min_progress:
            if current_time - self.stuck_timer > self.stuck_threshold:
                return True
        else:
            self.last_position = current_pos
            self.stuck_timer = current_time
            
        return False

    def escape_behavior(self):
        """Execute escape behavior when stuck"""
        print("Executing escape behavior!")
        # Reverse for a moment
        self.left_motor.setVelocity(-self.SAFE_SPEED)
        self.right_motor.setVelocity(-self.SAFE_SPEED)
        
        # After a short reverse, turn randomly
        if random.random() < 0.5:
            self.left_motor.setVelocity(self.TURN_SPEED)
            self.right_motor.setVelocity(-self.TURN_SPEED)
        else:
            self.left_motor.setVelocity(-self.TURN_SPEED)
            self.right_motor.setVelocity(self.TURN_SPEED)

    def calculate_motor_speeds(self, sensor_readings):
        """Calculate motor speeds based on sensor readings"""
        left_speed = self.SAFE_SPEED
        right_speed = self.SAFE_SPEED
        
        # Weight for each sensor based on its position
        weights = [
            [1.0, -1.0],    # Front-left
            [1.0, -1.0],    # Front-right
            [0.5, -1.0],    # Right
            [0.2, -0.5],    # Back-right
            [-0.2, -0.2],   # Back
            [-0.2, 0.5],    # Back-left
            [-1.0, 1.0],    # Left
            [-1.0, 1.0],    # Front-left
        ]
        
        # Calculate influence of each sensor
        for i, reading in enumerate(sensor_readings):
            if reading > self.CAUTION_THRESHOLD:
                # Normalize reading influence
                influence = (reading - self.CAUTION_THRESHOLD) / (self.DANGER_THRESHOLD - self.CAUTION_THRESHOLD)
                influence = min(1.0, influence)
                
                # Apply weighted influence to each motor
                left_speed += weights[i][0] * influence * self.TURN_SPEED
                right_speed += weights[i][1] * influence * self.TURN_SPEED
        
        # Ensure speeds are within bounds
        left_speed = max(-self.SAFE_SPEED, min(self.SAFE_SPEED, left_speed))
        right_speed = max(-self.SAFE_SPEED, min(self.SAFE_SPEED, right_speed))
        
        return left_speed, right_speed

    def update(self):
        """Main update loop for obstacle avoidance"""
        # Get current sensor readings
        sensor_readings = self.get_sensor_readings()
        
        # Update LED feedback
        self.update_leds(sensor_readings)
        
        # Check if stuck
        if self.check_if_stuck():
            self.escape_behavior()
            return
            
        # Calculate motor speeds based on sensor readings
        left_speed, right_speed = self.calculate_motor_speeds(sensor_readings)
        
        # Apply motor speeds
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
        # Print debug info
        max_reading = max(sensor_readings)
        if max_reading > self.CAUTION_THRESHOLD:
            print(f"Obstacle detected! Distance: {max_reading:.1f}")

    def enter(self):
        """Called when switching to obstacle avoidance mode"""
        print("Entering obstacle avoidance mode!")
        self.last_position = None
        self.stuck_timer = 0
        
    def exit(self):
        """Called when switching away from obstacle avoidance mode"""
        super().exit()
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)