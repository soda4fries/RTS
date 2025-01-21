# #!/Users/whakimi/myenv/bin/python
# # feeding_mode.py
# from base_mode import BaseMode
# import math
# import random

# class FeedingMode(BaseMode):
#     def __init__(self, robot):
#         super().__init__(robot)
#         # Constants
#         self.HUNGER_DECREASE = 2  # How much hunger decreases when eating
#         self.HUNGER_INCREASE = 0.1  # How much hunger increases over time
#         self.MAX_HUNGER = 100
#         self.MIN_HUNGER = 0
#         self.EATING_DISTANCE = 0.1  # Distance threshold to food
#         self.SPIN_SPEED = 2.0
        
#         # State variables
#         self.hunger = 50  # Start with medium hunger
#         self.food_position = None
#         self.is_eating = False
#         self.eating_timer = 0
#         self.EATING_DURATION = 3  # Seconds to spend eating
        
#     def initialize_devices(self):
#         super().initialize_devices()  # Call parent's initialize_devices first
        
#         # Initialize GPS and compass for navigation
#         self.gps = self.robot.getDevice('gps')
#         self.gps.enable(self.TIME_STEP)
        
#         self.compass = self.robot.getDevice('compass')
#         self.compass.enable(self.TIME_STEP)
        
#         # Initialize proximity sensors
#         self.ps = []
#         for i in range(8):
#             sensor = self.robot.getDevice(f'ps{i}')
#             if sensor:
#                 sensor.enable(self.TIME_STEP)
#                 self.ps.append(sensor)
                
#         # Initialize LEDs for feedback
#         self.leds = []
#         for i in range(8):
#             led = self.robot.getDevice(f'led{i}')
#             if led:
#                 self.leds.append(led)

#     def update_hunger(self):
#         """Update hunger level"""
#         if self.is_eating:
#             self.hunger = max(self.MIN_HUNGER, self.hunger - self.HUNGER_DECREASE)
#         else:
#             self.hunger = min(self.MAX_HUNGER, self.hunger + self.HUNGER_INCREASE)
            
#         # Visual feedback of hunger level using LEDs
#         led_threshold = (self.hunger / self.MAX_HUNGER) * len(self.leds)
#         for i, led in enumerate(self.leds):
#             led.set(1 if i < led_threshold else 0)

#     def detect_food(self):
#         """Simulate food detection using proximity sensors"""
#         # For simulation, we'll say food is detected when front sensors detect something
#         front_sensors = [self.ps[0], self.ps[7]]  # Front sensors
#         for sensor in front_sensors:
#             if sensor.getValue() > 100:  # Threshold for detection
#                 return True
#         return False

#     def calculate_distance(self, pos1, pos2):
#         """Calculate distance between two points"""
#         return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

#     def eating_behavior(self):
#         """Execute eating behavior"""
#         current_time = self.robot.getTime()
        
#         if not self.eating_timer:  # Start eating
#             self.eating_timer = current_time
#             print("Started eating!")
            
#         if current_time - self.eating_timer > self.EATING_DURATION:
#             self.is_eating = False
#             self.eating_timer = 0
#             print("Finished eating!")
#             return False
            
#         # Eating animation - gentle rocking motion
#         phase = (current_time * 2) % 2
#         if phase < 1:
#             self.left_motor.setVelocity(0.5)
#             self.right_motor.setVelocity(-0.5)
#         else:
#             self.left_motor.setVelocity(-0.5)
#             self.right_motor.setVelocity(0.5)
            
#         return True

#     def update(self):
#         """Main update loop for feeding mode"""
#         # Update hunger status
#         self.update_hunger()
#         print(f"Hunger level: {self.hunger:.1f}")
        
#         # If currently eating, continue the eating behavior
#         if self.is_eating:
#             still_eating = self.eating_behavior()
#             if not still_eating:
#                 self.is_eating = False
#             return
        
#         # Check for food
#         if self.detect_food():
#             print("Food detected!")
#             self.is_eating = True
#             return
            
#         # Search behavior - move around looking for food
#         if self.hunger > 70:  # Actively search when hungry
#             # Random wandering with a bias toward open areas
#             left_speed = 2.0
#             right_speed = 2.0
            
#             # Adjust direction based on sensor readings
#             for i, sensor in enumerate(self.ps):
#                 if sensor.getValue() > 80:  # Object detected
#                     if i < 4:  # Right side sensors
#                         left_speed += 1.0
#                         right_speed -= 1.0
#                     else:  # Left side sensors
#                         left_speed -= 1.0
#                         right_speed += 1.0
            
#             self.left_motor.setVelocity(left_speed)
#             self.right_motor.setVelocity(right_speed)
#         else:
#             # When not very hungry, move more slowly
#             self.left_motor.setVelocity(1.0)
#             self.right_motor.setVelocity(1.0)

#     def enter(self):
#         """Called when switching to feeding mode"""
#         print("Entering feeding mode!")
#         self.hunger = 50  # Reset hunger to medium
#         self.is_eating = False
#         self.eating_timer = 0
        
#     def exit(self):
#         """Called when switching away from feeding mode"""
#         super().exit()
#         # Turn off all LEDs
#         for led in self.leds:
#             led.set(0)

# # from base_mode import BaseMode
# # from camera_window import CameraWindow
# # import numpy as np
# # import math
# # import cv2
# # from collections import deque

# # class FeedingMode(BaseMode):
# #     def __init__(self, robot):
# #         super().__init__(robot)
# #         # Constants for movement
# #         self.TIME_STEP = 64
# #         self.SEARCH_SPEED = 1.0
# #         self.FORWARD_SPEED = 3.0
# #         self.TURN_FACTOR = 1.5
        
# #         # Constants for ball detection
# #         self.EATING_DISTANCE = 100
# #         self.MIN_RADIUS = 5
# #         self.MAX_RADIUS = 50
        
# #         # Improved HSV thresholds for white ball detection
# #         self.whiteLower = np.array([0, 0, 200])    # Increased value threshold
# #         self.whiteUpper = np.array([180, 30, 255]) # Full hue range, low saturation
        
# #         # Points deque for tracking
# #         self.pts = deque(maxlen=32)
        
# #         # State variables
# #         self.mood = 50
# #         self.is_eating = False
# #         self.ball_detected = False
# #         self.ball_position = None
        
# #         # Create camera window
# #         self.camera_window = CameraWindow()
        
# #     def initialize_devices(self):
# #         """Initialize robot devices"""
# #         super().initialize_devices()
        
# #         # Initialize camera
# #         self.camera = self.robot.getDevice('camera')
# #         if self.camera:
# #             self.camera.enable(self.TIME_STEP)
        
# #         # Initialize distance sensors
# #         self.ps = []
# #         for i in range(8):
# #             sensor = self.robot.getDevice(f'ps{i}')
# #             if sensor:
# #                 sensor.enable(self.TIME_STEP)
# #                 self.ps.append(sensor)
        
# #         # Initialize LEDs
# #         self.leds = []
# #         for i in range(8):
# #             led = self.robot.getDevice(f'led{i}')
# #             if led:
# #                 self.leds.append(led)
    
# #     def detect_ball(self):
# #         """Enhanced ball detection using HSV color space and contour analysis"""
# #         # Get camera image
# #         img = self.camera.getImage()
# #         if img is None:
# #             print("No camera image available")
# #             return False, None
            
# #         # Convert to numpy array
# #         width = self.camera.getWidth()
# #         height = self.camera.getHeight()
# #         img_array = np.frombuffer(img, np.uint8).reshape((height, width, 4))
        
# #         # Convert BGRA to BGR
# #         frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
        
# #         # Create copy for visualization
# #         output = frame.copy()
        
# #         # Apply Gaussian blur to reduce noise
# #         blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        
# #         # Convert to HSV color space
# #         hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
# #         # Create mask for white color
# #         mask = cv2.inRange(hsv, self.whiteLower, self.whiteUpper)
        
# #         # Perform morphological operations to remove small blobs
# #         kernel = np.ones((5,5), np.uint8)
# #         mask = cv2.erode(mask, kernel, iterations=2)
# #         mask = cv2.dilate(mask, kernel, iterations=2)
        
# #         # Find contours in the mask
# #         contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
# #             cv2.CHAIN_APPROX_SIMPLE)
        
# #         # Initialize ball detection variables
# #         ball_detected = False
# #         ball_position = None
        
# #         # Process contours if any found
# #         if len(contours) > 0:
# #             # Find the largest contour
# #             c = max(contours, key=cv2.contourArea)
# #             area = cv2.contourArea(c)
            
# #             # Get the minimum enclosing circle
# #             ((x, y), radius) = cv2.minEnclosingCircle(c)
            
# #             # Calculate circularity
# #             perimeter = cv2.arcLength(c, True)
# #             circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
            
# #             # Check if the contour meets our criteria
# #             if (radius > self.MIN_RADIUS and 
# #                 radius < self.MAX_RADIUS and 
# #                 circularity > 0.8):  # High circularity threshold
                
# #                 # Calculate center using moments
# #                 M = cv2.moments(c)
# #                 if M["m00"] > 0:
# #                     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    
# #                     # Draw the circle and centroid
# #                     cv2.circle(output, (int(x), int(y)), int(radius),
# #                         (0, 255, 255), 2)
# #                     cv2.circle(output, center, 5, (0, 0, 255), -1)
                    
# #                     # Calculate ball position relative to robot
# #                     ball_x = (x - width/2) * 0.01
# #                     ball_y = (height - y) * 0.01
# #                     ball_position = (ball_x, ball_y)
# #                     ball_detected = True
                    
# #                     # Update tracking points
# #                     self.pts.appendleft(center)
                    
# #                     # Draw tracking trail
# #                     for i in range(1, len(self.pts)):
# #                         if self.pts[i - 1] is None or self.pts[i] is None:
# #                             continue
# #                         thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
# #                         cv2.line(output, self.pts[i - 1], self.pts[i], 
# #                                 (0, 0, 255), thickness)
                    
# #                     # Update status
# #                     self.camera_window.status_var.set(
# #                         f"Ball detected - Position: ({int(x)}, {int(y)}), "
# #                         f"Radius: {int(radius)}, Circularity: {circularity:.2f}")
        
# #         if not ball_detected:
# #             self.camera_window.status_var.set("Searching for ball...")
# #             self.pts.clear()  # Clear tracking points when ball is lost
        
# #         # Update display based on view mode
# #         if self.camera_window.view_mode.get() == "mask":
# #             display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
# #         else:
# #             display_img = output
            
# #         self.camera_window.update_image(display_img)
        
# #         return ball_detected, ball_position
        
# #     def check_eating_distance(self):
# #         """Check if close enough to eat using front sensors"""
# #         front_sensors = [self.ps[0], self.ps[7]]  # Front sensors
# #         return any(sensor.getValue() > self.EATING_DISTANCE for sensor in front_sensors)
        
# #     def calculate_motor_speeds(self, ball_position):
# #         """Calculate motor speeds to move towards ball"""
# #         if not ball_position:
# #             return self.SEARCH_SPEED, -self.SEARCH_SPEED
            
# #         x, y = ball_position
        
# #         # If ball is roughly centered
# #         if abs(x) < 0.1:
# #             return self.FORWARD_SPEED, self.FORWARD_SPEED
# #         else:
# #             # Turn towards ball with smoother control
# #             turn_amount = self.TURN_FACTOR * x
# #             if x > 0:  # Ball is to the right
# #                 return self.FORWARD_SPEED, self.FORWARD_SPEED - turn_amount
# #             else:  # Ball is to the left
# #                 return self.FORWARD_SPEED - abs(turn_amount), self.FORWARD_SPEED
                
# #     def update_leds(self):
# #         """Update LED pattern based on state"""
# #         if self.is_eating:
# #             # Spin pattern when eating
# #             time = int(self.robot.getTime() * 2)
# #             led_index = time % len(self.leds)
# #             for i, led in enumerate(self.leds):
# #                 led.set(1 if i == led_index else 0)
# #         elif self.ball_detected:
# #             # Front LEDs blink when ball detected
# #             time = int(self.robot.getTime() * 4)
# #             state = time % 2
# #             self.leds[0].set(state)
# #             self.leds[7].set(state)
# #         else:
# #             # LEDs indicate search mode
# #             time = int(self.robot.getTime() * 1)
# #             led_index = time % len(self.leds)
# #             for i, led in enumerate(self.leds):
# #                 led.set(1 if i == led_index else 0)
                
# #     def update(self):
# #         """Main update loop"""
# #         try:
# #             # Update camera window
# #             self.camera_window.update()
            
# #             # Detect ball
# #             self.ball_detected, self.ball_position = self.detect_ball()
            
# #             # Check if in eating position
# #             if self.ball_detected and self.check_eating_distance():
# #                 self.is_eating = True
# #                 # Stop when eating
# #                 self.left_motor.setVelocity(0)
# #                 self.right_motor.setVelocity(0)
# #             else:
# #                 self.is_eating = False
# #                 # Move towards ball or search
# #                 left_speed, right_speed = self.calculate_motor_speeds(self.ball_position)
# #                 self.left_motor.setVelocity(left_speed)
# #                 self.right_motor.setVelocity(right_speed)
            
# #             # Update LEDs
# #             self.update_leds()
                
# #         except Exception as e:
# #             print(f"Error in feeding mode update: {e}")
            
# #     def enter(self):
# #         """Initialize when entering feeding mode"""
# #         print("Entering feeding mode!")
# #         self.is_eating = False
# #         self.ball_detected = False
# #         self.ball_position = None
        
# #     def exit(self):
# #         """Cleanup when exiting feeding mode"""
# #         super().exit()
# #         self.camera_window.close()
# #         for led in self.leds:
# #             led.set(0)

#!/Users/whakimi/myenv/bin/python
# feeding_mode.py
from base_mode import BaseMode
import random
import math

class FeedingMode(BaseMode):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants
        self.MAX_HUNGER = 100
        self.MIN_HUNGER = 0
        self.HUNGER_INCREASE_RATE = 0.1  # Rate at which hunger increases
        self.FOOD_VALUE = 20  # Amount hunger decreases when fed
        self.DETECTION_THRESHOLD = 1000  # Threshold for detecting "food" (proximity sensor)
        self.SATISFIED_THRESHOLD = 30  # Threshold for being satisfied
        self.HUNGRY_THRESHOLD = 70  # Threshold for being hungry
        self.SPIN_SPEED = 2.0
        
        # State variables
        self.hunger_level = 50
        self.last_feed_time = 0
        self.feeding_cooldown = 3  # Seconds between feeds
        self.is_eating = False
        self.eating_start_time = 0
        self.eating_duration = 2  # Seconds to perform eating animation
        self.searching_for_food = False
        
    def initialize_devices(self):
        """Initialize devices specific to feeding mode"""
        super().initialize_devices()
        
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
        """Update hunger level over time"""
        self.hunger_level = min(self.MAX_HUNGER, 
                              self.hunger_level + self.HUNGER_INCREASE_RATE)
        if self.hunger_level > self.HUNGRY_THRESHOLD:
            self.searching_for_food = True
        elif self.hunger_level < self.SATISFIED_THRESHOLD:
            self.searching_for_food = False

    def detect_food(self):
        """Check proximity sensors for "food" (objects close to sensors)"""
        current_time = self.robot.getTime()
        if current_time - self.last_feed_time < self.feeding_cooldown:
            return False

        # Check front sensors (0, 7) for food detection
        front_sensors = [self.ps[0], self.ps[7]]
        for sensor in front_sensors:
            if sensor.getValue() > self.DETECTION_THRESHOLD:
                return True
        return False

    def eat_animation(self):
        """Perform eating animation"""
        current_time = self.robot.getTime()
        if not self.is_eating:
            self.is_eating = True
            self.eating_start_time = current_time
            # Blink LEDs to show eating
            for led in self.leds:
                led.set(1)
        
        # Check if eating animation should end
        if current_time - self.eating_start_time > self.eating_duration:
            self.is_eating = False
            self.last_feed_time = current_time
            # Turn off LEDs
            for led in self.leds:
                led.set(0)
            # Reduce hunger
            self.hunger_level = max(self.MIN_HUNGER, 
                                  self.hunger_level - self.FOOD_VALUE)
            return False
        return True

    def search_for_food(self):
        """Behavior when searching for food"""
        # Random movement pattern when searching for food
        if random.random() < 0.02:  # 2% chance to change direction
            self.left_motor.setVelocity(random.uniform(-self.SPIN_SPEED, self.SPIN_SPEED))
            self.right_motor.setVelocity(random.uniform(-self.SPIN_SPEED, self.SPIN_SPEED))
        else:
            # Move forward with slight random adjustments
            base_speed = self.SPIN_SPEED * 0.75
            self.left_motor.setVelocity(base_speed + random.uniform(-0.5, 0.5))
            self.right_motor.setVelocity(base_speed + random.uniform(-0.5, 0.5))

    def hungry_behavior(self):
        """Display hungry behavior through LED patterns"""
        current_time = self.robot.getTime()
        blink_period = 1.0  # seconds
        # Blink LEDs faster when hungrier
        if self.hunger_level > self.HUNGRY_THRESHOLD:
            blink_period = 0.5
        
        # Alternate LEDs based on time
        should_be_on = int(current_time / blink_period) % 2 == 0
        for i, led in enumerate(self.leds):
            if i % 2 == 0:
                led.set(1 if should_be_on else 0)
            else:
                led.set(0 if should_be_on else 1)

    def display_hunger_level(self):
        """Display hunger level using LEDs"""
        # Calculate how many LEDs should be on based on hunger level
        num_leds = len(self.leds)
        leds_to_light = int((self.hunger_level / self.MAX_HUNGER) * num_leds)
        
        # Update LEDs
        for i in range(num_leds):
            self.leds[i].set(1 if i < leds_to_light else 0)

    def update(self):
        """Main update loop for feeding mode"""
        # Update hunger over time
        self.update_hunger()
        
        # Check if currently performing eating animation
        if self.is_eating:
            if self.eat_animation():
                return  # Continue eating animation
        
        # Check for food if not eating
        if self.detect_food():
            # Start eating animation
            self.eat_animation()
        elif self.searching_for_food:
            # Search for food if hungry
            self.search_for_food()
            self.hungry_behavior()
        else:
            # Display contentment through steady LED pattern
            self.display_hunger_level()
            # Stop motors when not searching
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)

    def enter(self):
        """Called when switching to feeding mode"""
        print("Entering feeding mode!")
        self.hunger_level = 50  # Reset hunger to middle value
        self.last_feed_time = self.robot.getTime()
        self.is_eating = False
        self.searching_for_food = False

    def exit(self):
        """Called when switching away from feeding mode"""
        super().exit()  # Call parent exit to stop motors
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)