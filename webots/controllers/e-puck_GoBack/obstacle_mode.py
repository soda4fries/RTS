#!/Users/whakimi/myenv/bin/python
# obstacle_mode.py
from base_mode import BaseMode
import math

class ObstacleMode(BaseMode):
    def __init__(self, robot):
        # Initialize state variables before calling parent constructor
        self.MAX_SPEED = 6.28
        self.OBSTACLE_THRESHOLD = 80.0
        self.LED_DURATION = 100  # Duration for LED feedback in milliseconds
        self.led_states = {}  # Initialize led_states dictionary
        self.current_state = "moving"  # Current movement state
        self.turn_direction = None  # Track turning direction for LED patterns
        self.ps = []  # Initialize proximity sensors list
        self.leds = []  # Initialize LEDs list
        
        # Now call parent constructor
        super().__init__(robot)
        
    def initialize_devices(self):
        """Initialize devices specific to obstacle mode"""
        super().initialize_devices()
        
        # Initialize proximity sensors
        self.ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        for i in range(8):
            sensor = self.robot.getDevice(self.ps_names[i])
            sensor.enable(self.TIME_STEP)
            self.ps.append(sensor)
            
        # Initialize LEDs for visual feedback
        for i in range(8):
            led = self.robot.getDevice(f'led{i}')
            if led:
                self.leds.append(led)
                self.led_states[i] = 0  # Initialize LED timestamps
    
    def update_leds(self, left_obstacle, right_obstacle):
        """Update LEDs based on obstacle detection and movement"""
        current_time = self.robot.getTime() * 1000  # Convert to milliseconds
        
        # Reset all LEDs first
        for led in self.leds:
            led.set(0)
            
        if left_obstacle or right_obstacle:
            # Light up LEDs on the side where obstacles are detected
            if left_obstacle:
                self.turn_direction = "right"
                # Light up left side LEDs (5, 6, 7)
                for i in [5, 6, 7]:
                    if i < len(self.leds):  # Check if LED exists
                        self.leds[i].set(1)
                        self.led_states[i] = current_time
                    
            if right_obstacle:
                self.turn_direction = "left"
                # Light up right side LEDs (0, 1, 2)
                for i in [0, 1, 2]:
                    if i < len(self.leds):  # Check if LED exists
                        self.leds[i].set(1)
                        self.led_states[i] = current_time
        else:
            # No obstacles - create "moving forward" pattern
            self.turn_direction = None
            # Alternate front and back LEDs
            phase = int((current_time / 500) % 2)  # 500ms period
            if phase == 0 and len(self.leds) > 7:  # Check if LEDs exist
                # Light up front LEDs (0, 7)
                self.leds[0].set(1)
                self.leds[7].set(1)
            elif len(self.leds) > 4:  # Check if LEDs exist
                # Light up back LEDs (3, 4)
                self.leds[3].set(1)
                self.leds[4].set(1)
    
    def create_turning_pattern(self):
        """Create LED pattern while turning"""
        if not self.leds:  # If no LEDs available, return
            return
            
        current_time = self.robot.getTime() * 1000
        if self.turn_direction == "right":
            # Rotating pattern clockwise
            led_index = int((current_time / 100) % len(self.leds))
            self.leds[led_index].set(1)
        elif self.turn_direction == "left":
            # Rotating pattern counter-clockwise
            led_index = (len(self.leds) - 1) - int((current_time / 100) % len(self.leds))
            self.leds[led_index].set(1)
    
    def update(self):
        """Main update loop for obstacle mode"""
        # Read sensor values
        ps_values = [sensor.getValue() for sensor in self.ps]
        
        # Detect obstacles
        right_obstacle = any(val > self.OBSTACLE_THRESHOLD for val in ps_values[0:3])
        left_obstacle = any(val > self.OBSTACLE_THRESHOLD for val in ps_values[5:8])
        
        # Initialize motor speeds at 50% of MAX_SPEED
        left_speed = 0.5 * self.MAX_SPEED
        right_speed = 0.5 * self.MAX_SPEED
        
        # Modify speeds according to obstacles
        if left_obstacle:
            # Turn right
            left_speed += 0.5 * self.MAX_SPEED
            right_speed -= 0.5 * self.MAX_SPEED
            self.current_state = "turning_right"
        elif right_obstacle:
            # Turn left
            left_speed -= 0.5 * self.MAX_SPEED
            right_speed += 0.5 * self.MAX_SPEED
            self.current_state = "turning_left"
        else:
            self.current_state = "moving"
        
        # Update visual feedback
        self.update_leds(left_obstacle, right_obstacle)
        
        # If turning, create dynamic LED pattern
        if self.current_state in ["turning_right", "turning_left"]:
            self.create_turning_pattern()
        
        # Set motor velocities
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def enter(self):
        """Called when switching to obstacle mode"""
        print("Entering obstacle avoidance mode!")
        # Reset all LEDs
        for led in self.leds:
            led.set(0)
        self.current_state = "moving"
        self.turn_direction = None
    
    def exit(self):
        """Called when switching away from obstacle mode"""
        super().exit()  # Call parent exit to stop motors
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)