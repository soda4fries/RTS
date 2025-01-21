# #!/Users/whakimi/myenv/bin/python
# # follow_mode.py
# from base_mode import BaseMode
# import math
# from math import atan2, degrees

# class FollowMode(BaseMode):
#     def __init__(self, robot):
#         super().__init__(robot)
#         # Constants
#         self.FOLLOW_DISTANCE = 80    # Ideal distance to maintain from target
#         self.CLOSE_THRESHOLD = 60    # Distance considered too close
#         self.FAR_THRESHOLD = 100     # Distance considered too far
#         self.MAX_SPEED = 4.0
#         self.TURN_SPEED = 2.0
        
#         # State tracking
#         self.target_found = False
#         self.search_direction = 1  # 1 or -1 for search direction
#         self.lost_target_timer = 0
#         self.SEARCH_TIME = 5.0  # Time to search in one direction before switching
        
#     def initialize_devices(self):
#         super().initialize_devices()  # Call parent's initialize_devices first
        
#         # Initialize distance sensors
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

#     def detect_target(self):
#         """
#         Detect target using proximity sensors
#         Returns: (found, distance, angle)
#         """
#         # Read all sensors
#         readings = [sensor.getValue() for sensor in self.ps]
#         max_reading = max(readings)
        
#         if max_reading > 0:
#             # Target found - determine its relative position
#             strongest_sensor = readings.index(max_reading)
            
#             # Calculate approximate angle (-180 to 180 degrees)
#             # Front sensor is 0 degrees
#             angle = (strongest_sensor * 45) % 360
#             if angle > 180:
#                 angle -= 360
                
#             return True, max_reading, angle
            
#         return False, 0, 0

#     def update_leds(self, target_found, target_angle=None):
#         """Update LEDs based on target detection status"""
#         if target_found:
#             # Light up LEDs in the direction of the target
#             led_index = int((target_angle + 180) / 45) % 8
#             for i, led in enumerate(self.leds):
#                 led.set(1 if i == led_index else 0)
#         else:
#             # When searching, create a moving light pattern
#             time = self.robot.getTime()
#             led_index = int(time * 2) % len(self.leds)
#             for i, led in enumerate(self.leds):
#                 led.set(1 if i == led_index else 0)

#     def search_behavior(self):
#         """Execute search pattern when target is lost"""
#         current_time = self.robot.getTime()
        
#         # Switch direction periodically
#         if current_time - self.lost_target_timer > self.SEARCH_TIME:
#             self.search_direction *= -1
#             self.lost_target_timer = current_time
            
#         # Spin in place while searching
#         self.left_motor.setVelocity(self.TURN_SPEED * self.search_direction)
#         self.right_motor.setVelocity(-self.TURN_SPEED * self.search_direction)

#     def calculate_follow_speeds(self, distance, angle):
#         """Calculate motor speeds for following the target"""
#         # Base speeds - moving forward
#         left_speed = self.MAX_SPEED
#         right_speed = self.MAX_SPEED
        
#         # Adjust speeds based on angle to target
#         # Convert angle to radians for math
#         angle_rad = math.radians(angle)
        
#         # Apply angular adjustment
#         angle_factor = math.cos(angle_rad)
#         turn_amount = math.sin(angle_rad)
        
#         # Distance factor - slow down when close, speed up when far
#         distance_factor = 1.0
#         if distance < self.CLOSE_THRESHOLD:
#             # Too close - back up
#             distance_factor = -0.5
#         elif distance < self.FOLLOW_DISTANCE:
#             # Slightly close - slow down
#             distance_factor = 0.5
#         elif distance > self.FAR_THRESHOLD:
#             # Too far - speed up
#             distance_factor = 1.5
            
#         # Apply factors to base speeds
#         base_speed = self.MAX_SPEED * distance_factor * angle_factor
#         turn_speed = self.TURN_SPEED * turn_amount
        
#         left_speed = base_speed + turn_speed
#         right_speed = base_speed - turn_speed
        
#         # Ensure speeds are within bounds
#         left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
#         right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))
        
#         return left_speed, right_speed

#     def update(self):
#         """Main update loop for following behavior"""
#         # Try to detect target
#         target_found, distance, angle = self.detect_target()
        
#         # Update LED display
#         self.update_leds(target_found, angle if target_found else None)
        
#         if target_found:
#             # Reset search state
#             self.lost_target_timer = self.robot.getTime()
            
#             # Calculate and apply motor speeds for following
#             left_speed, right_speed = self.calculate_follow_speeds(distance, angle)
#             self.left_motor.setVelocity(left_speed)
#             self.right_motor.setVelocity(right_speed)
            
#             # Debug output
#             print(f"Following target - Distance: {distance:.1f}, Angle: {angle:.1f}")
            
#         else:
#             # Target lost - execute search behavior
#             self.search_behavior()
#             print("Searching for target...")

#     def enter(self):
#         """Called when switching to follow mode"""
#         print("Entering follow mode!")
#         self.target_found = False
#         self.lost_target_timer = self.robot.getTime()
#         self.search_direction = 1
        
#     def exit(self):
#         """Called when switching away from follow mode"""
#         super().exit()
#         # Turn off all LEDs
#         for led in self.leds:
#             led.set(0)
#

#!/Users/whakimi/myenv/bin/python
# follow_mode.py
from base_mode import BaseMode
import math
import numpy as np
import cv2
from collections import deque

class FollowMode(BaseMode):
    def __init__(self, robot):
        # Initialize state variables
        self.MAX_SPEED = 6.28
        self.MIN_SPEED = 0.0
        self.TARGET_DISTANCE = 100
        self.camera_enabled = False
        self.led_states = {}
        self.search_direction = 1
        self.search_time = 0
        
        # Ball tracking parameters for white ball
        # HSV values for white: low saturation, high value
        self.whiteLower = (0, 0, 200)  # Lower bound for white in HSV
        self.whiteUpper = (180, 30, 255)  # Upper bound for white in HSV
        self.pts = deque(maxlen=32)  # Tracking points buffer
        self.min_radius = 2  # Minimum ball radius to track
        self.last_ball_pos = None
        
        # Debug window name
        self.debug_window = "E-puck Camera View"
        
        super().__init__(robot)

    def initialize_devices(self):
        """Initialize devices specific to follow mode"""
        super().initialize_devices()
        
        # Initialize camera
        try:
            self.camera = self.robot.getDevice("camera")
            if self.camera:
                self.camera.enable(self.TIME_STEP)
                self.camera_width = self.camera.getWidth()
                self.camera_height = self.camera.getHeight()
                self.camera_enabled = True
                print(f"Camera initialized: {self.camera_width}x{self.camera_height}")
            else:
                print("Camera not found")
        except Exception as e:
            print(f"Error initializing camera: {e}")
            self.camera_enabled = False
        
        # Initialize LEDs
        self.leds = []
        for i in range(8):
            try:
                led = self.robot.getDevice(f'led{i}')
                if led:
                    self.leds.append(led)
                    self.led_states[i] = 0
            except Exception as e:
                print(f"Error initializing LED {i}: {e}")

    def detect_ball(self):
        """Detect and track a green ball using the camera"""
        if not self.camera_enabled:
            return None, None, None

        try:
            # Get camera image
            image = self.camera.getImage()
            if image is None:
                return None, None, None

            # Convert image to numpy array
            frame = np.frombuffer(image, np.uint8).reshape((self.camera_height, self.camera_width, 4))
            
            # Convert to BGR format (OpenCV format)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Blur the frame and convert to HSV color space
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # Construct a mask for white color
            mask = cv2.inRange(hsv, self.whiteLower, self.whiteUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Initialize ball position
            ball_pos = None
            ball_radius = 0
            center = None
            
            # Only proceed if at least one contour was found
            if len(contours) > 0:
                # Find the largest contour
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                
                if M["m00"] > 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    
                    # Only proceed if the radius meets a minimum size
                    if radius > self.min_radius:
                        ball_pos = (int(x), int(y))
                        ball_radius = int(radius)
                        
                        # Draw circle and centroid for debugging
                        cv2.circle(frame, ball_pos, ball_radius, (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        print(f"Ball detected at position: {ball_pos}, radius: {ball_radius}")
            
            # Update tracking points
            self.pts.appendleft(center)
            
            # Draw tracking trail
            for i in range(1, len(self.pts)):
                if self.pts[i - 1] is None or self.pts[i] is None:
                    continue
                thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
                cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
            
            # Show debug window
            cv2.imshow(self.debug_window, frame)
            cv2.waitKey(1)
            
            return ball_pos, ball_radius, frame
            
        except Exception as e:
            print(f"Error in ball detection: {e}")
            return None, None, None

    def update_leds_following(self, ball_pos):
        """Update LEDs based on ball position"""
        if not self.leds or not ball_pos:
            return
            
        try:
            # Reset all LEDs
            for led in self.leds:
                led.set(0)
                
            x = ball_pos[0]
            if x < self.camera_width * 0.4:  # Ball is on the left
                for i in [5, 6, 7]:  # Left side LEDs
                    if i < len(self.leds):
                        self.leds[i].set(1)
            elif x > self.camera_width * 0.6:  # Ball is on the right
                for i in [0, 1, 2]:  # Right side LEDs
                    if i < len(self.leds):
                        self.leds[i].set(1)
            else:  # Ball is centered
                if len(self.leds) > 7:
                    self.leds[0].set(1)
                    self.leds[7].set(1)
        except Exception as e:
            print(f"Error updating LEDs: {e}")

    def search_pattern(self):
        """Execute search pattern when ball is not visible"""
        try:
            current_time = self.robot.getTime()
            
            # Change direction every 4 seconds
            if current_time - self.search_time > 4.0:
                self.search_direction *= -1
                self.search_time = current_time
                print("Changing search direction")
            
            # Rotate in place
            search_speed = 2.0
            self.left_motor.setVelocity(search_speed * self.search_direction)
            self.right_motor.setVelocity(-search_speed * self.search_direction)
            
            # Rotating LED pattern
            led_index = int((current_time * 1000) / 200) % len(self.leds)
            for i, led in enumerate(self.leds):
                led.set(1 if i == led_index else 0)
                
        except Exception as e:
            print(f"Error in search pattern: {e}")
            self.stop_motors()

    def follow_ball(self, ball_pos, ball_radius):
        """Calculate motor speeds to follow ball"""
        try:
            # Calculate horizontal error (-1 to 1)
            center_x = self.camera_width / 2
            error_x = (ball_pos[0] - center_x) / center_x
            
            # Base speeds
            left_speed = 0.5 * self.MAX_SPEED
            right_speed = 0.5 * self.MAX_SPEED
            
            # Adjust speeds based on horizontal position
            turn_factor = 0.5 * error_x
            left_speed -= self.MAX_SPEED * turn_factor
            right_speed += self.MAX_SPEED * turn_factor
            
            # Adjust speed based on ball size (distance)
            size_factor = (ball_radius - self.TARGET_DISTANCE/10) / (self.TARGET_DISTANCE/10)
            speed_scale = 1.0 - min(1.0, abs(size_factor))
            
            # Apply speeds
            left_speed = max(min(left_speed * speed_scale, self.MAX_SPEED), self.MIN_SPEED)
            right_speed = max(min(right_speed * speed_scale, self.MAX_SPEED), self.MIN_SPEED)
            
            print(f"Following ball - Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            # Update LEDs
            self.update_leds_following(ball_pos)
            
        except Exception as e:
            print(f"Error in follow_ball: {e}")
            self.stop_motors()

    def stop_motors(self):
        """Safety function to stop motors"""
        try:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def update(self):
        """Main update loop for follow mode"""
        try:
            # Detect ball
            ball_pos, ball_radius, _ = self.detect_ball()
            
            if ball_pos is not None and ball_radius > 0:
                # Ball found - follow it
                self.last_ball_pos = ball_pos
                self.follow_ball(ball_pos, ball_radius)
            else:
                # No ball - search for it
                print("No ball detected, searching...")
                self.search_pattern()
                
        except Exception as e:
            print(f"Error in update loop: {e}")
            self.stop_motors()

    def enter(self):
        """Called when switching to follow mode"""
        print("Entering follow mode!")
        cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
        self.search_time = self.robot.getTime()
        # Reset all LEDs
        if self.leds:
            for led in self.leds:
                led.set(0)

    def exit(self):
        """Called when switching away from follow mode"""
        print("Exiting follow mode!")
        try:
            super().exit()  # Call parent exit to stop motors
            # Turn off all LEDs
            if self.leds:
                for led in self.leds:
                    led.set(0)
            # Clean up OpenCV windows
            cv2.destroyWindow(self.debug_window)
        except Exception as e:
            print(f"Error in exit: {e}")