# feeding_mode.py
from base_mode import BaseMode
import math
import random

class FeedingMode(BaseMode):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants
        self.HUNGER_DECREASE = 2  # How much hunger decreases when eating
        self.HUNGER_INCREASE = 0.1  # How much hunger increases over time
        self.MAX_HUNGER = 100
        self.MIN_HUNGER = 0
        self.EATING_DISTANCE = 0.1  # Distance threshold to food
        self.SPIN_SPEED = 2.0
        
        # State variables
        self.hunger = 50  # Start with medium hunger
        self.food_position = None
        self.is_eating = False
        self.eating_timer = 0
        self.EATING_DURATION = 3  # Seconds to spend eating
        
    def initialize_devices(self):
        super().initialize_devices()  # Call parent's initialize_devices first
        
        # Initialize GPS and compass for navigation
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.TIME_STEP)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.TIME_STEP)
        
        # Initialize proximity sensors
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

    def update_hunger(self):
        """Update hunger level"""
        if self.is_eating:
            self.hunger = max(self.MIN_HUNGER, self.hunger - self.HUNGER_DECREASE)
        else:
            self.hunger = min(self.MAX_HUNGER, self.hunger + self.HUNGER_INCREASE)
            
        # Visual feedback of hunger level using LEDs
        led_threshold = (self.hunger / self.MAX_HUNGER) * len(self.leds)
        for i, led in enumerate(self.leds):
            led.set(1 if i < led_threshold else 0)

    def detect_food(self):
        """Simulate food detection using proximity sensors"""
        # For simulation, we'll say food is detected when front sensors detect something
        front_sensors = [self.ps[0], self.ps[7]]  # Front sensors
        for sensor in front_sensors:
            if sensor.getValue() > 100:  # Threshold for detection
                return True
        return False

    def calculate_distance(self, pos1, pos2):
        """Calculate distance between two points"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def eating_behavior(self):
        """Execute eating behavior"""
        current_time = self.robot.getTime()
        
        if not self.eating_timer:  # Start eating
            self.eating_timer = current_time
            print("Started eating!")
            
        if current_time - self.eating_timer > self.EATING_DURATION:
            self.is_eating = False
            self.eating_timer = 0
            print("Finished eating!")
            return False
            
        # Eating animation - gentle rocking motion
        phase = (current_time * 2) % 2
        if phase < 1:
            self.left_motor.setVelocity(0.5)
            self.right_motor.setVelocity(-0.5)
        else:
            self.left_motor.setVelocity(-0.5)
            self.right_motor.setVelocity(0.5)
            
        return True

    def update(self):
        """Main update loop for feeding mode"""
        # Update hunger status
        self.update_hunger()
        print(f"Hunger level: {self.hunger:.1f}")
        
        # If currently eating, continue the eating behavior
        if self.is_eating:
            still_eating = self.eating_behavior()
            if not still_eating:
                self.is_eating = False
            return
        
        # Check for food
        if self.detect_food():
            print("Food detected!")
            self.is_eating = True
            return
            
        # Search behavior - move around looking for food
        if self.hunger > 70:  # Actively search when hungry
            # Random wandering with a bias toward open areas
            left_speed = 2.0
            right_speed = 2.0
            
            # Adjust direction based on sensor readings
            for i, sensor in enumerate(self.ps):
                if sensor.getValue() > 80:  # Object detected
                    if i < 4:  # Right side sensors
                        left_speed += 1.0
                        right_speed -= 1.0
                    else:  # Left side sensors
                        left_speed -= 1.0
                        right_speed += 1.0
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
        else:
            # When not very hungry, move more slowly
            self.left_motor.setVelocity(1.0)
            self.right_motor.setVelocity(1.0)

    def enter(self):
        """Called when switching to feeding mode"""
        print("Entering feeding mode!")
        self.hunger = 50  # Reset hunger to medium
        self.is_eating = False
        self.eating_timer = 0
        
    def exit(self):
        """Called when switching away from feeding mode"""
        super().exit()
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)