#!/usr/bin/env python
# petting_mode.py
from base_mode import BaseMode
import math
from math import atan2, degrees
import random

class PettingMode(BaseMode):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants
        self.SPIN_SPEED = 2.0
        self.destination = (0, 0)  # Default destination at origin
        self.MOOD_THRESHOLD_HIGH = 55
        self.MOOD_THRESHOLD_LOW = 25
        self.MOOD_INCREASE = 5
        self.MOOD_DECREASE = 0.5
        self.LED_DURATION = 3000  # LED blink duration in milliseconds

        # State variables
        self.mood = 50  # Initial mood value
        self.dance_flag = False
        self.dance_state_persistent = False
        self.current_time = 0
        self.dance_interrupted_time = 0
        self.has_meowed = False  # Track if meow has played

        # Direction mapping for debug messages
        self.direction_names = {
            0: "FRONT",
            1: "FRONT-RIGHT",
            2: "RIGHT",
            3: "BACK-RIGHT",
            4: "BACK",
            5: "BACK-LEFT",
            6: "LEFT",
            7: "FRONT-LEFT"
        }

    def initialize_devices(self):
        """Initialize all robot devices and sensors"""
        super().initialize_devices()
        
        # Initialize sound device
        self.speaker = self.robot.getDevice('speaker')
        
        # Initialize compass
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.TIME_STEP)
        
        # Initialize GPS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.TIME_STEP)
        
        # Initialize LEDs
        self.leds = []
        for i in range(8):
            led = self.robot.getDevice(f'led{i}')
            if led:
                self.leds.append(led)
        
        # Initialize proximity sensors
        self.ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            if sensor:
                sensor.enable(self.TIME_STEP)
                self.ps.append(sensor)
        
        # LED timer dictionary to track blink durations
        self.led_timers = {i: 0 for i in range(len(self.leds))}

    def check_collisions(self):
        """Check for nearby objects using distance sensors"""
        close_threshold = 1000
        near_threshold = 100
        collisions = []
        has_interaction = False
        
        for i, sensor in enumerate(self.ps):
            value = sensor.getValue()
            if value > close_threshold:
                print(f"Very close object detected at {self.direction_names[i]}")
                collisions.append(i)
                self.blink_led(i)
                has_interaction = True
            elif value > near_threshold:
                print(f"Nearby object detected at {self.direction_names[i]}")
                has_interaction = True
        
        return collisions, has_interaction

    def update_mood(self, has_interaction):
        """Update mood based on interactions"""
        old_mood = self.mood
        
        if has_interaction:
            self.mood = min(100, self.mood + self.MOOD_INCREASE)
        else:
            self.mood = max(0, self.mood - self.MOOD_DECREASE)
        
        # Play meow sound when mood reaches 100
        if self.mood == 100 and old_mood < 100 and not self.has_meowed:
            try:
                # Simplified sound playback that won't crash the simulation
                self.speaker.playSound(self.speaker, self.speaker, 'sounds/meow.wav', 1.0, 1.0, 0.0, False)
                self.has_meowed = True
                print("Playing meow sound!")
            except Exception as e:
                print(f"Error playing sound: {e}")
                # Don't let sound errors affect the simulation
                pass
        
        # Reset meow flag if mood drops below 100
        if self.mood < 100:
            self.has_meowed = False
        
        print(f"Current mood: {self.mood}")
        return self.mood

    def mood_check(self):
        """Update LED states based on mood"""
        if self.mood > self.MOOD_THRESHOLD_HIGH:
            for led in self.leds:
                if led.get() == 1:
                    self.blinks_led(led, True)
                else:
                    self.blinks_led(led, False)
        else:
            for led in self.leds:
                self.blinks_led(led, True)

    def blinks_led(self, led, isON):
        """Set LED state"""
        if led is not None:
            if isON:
                led.set(0)
            else:
                led.set(1)

    def blink_led(self, led_index):
        """Start LED blink sequence"""
        if 0 <= led_index < len(self.leds):
            current_time = self.robot.getTime() * 1000
            self.leds[led_index].set(1)
            self.led_timers[led_index] = current_time
            print(f"LED {led_index} turned ON ({self.direction_names[led_index]} position)")

    def update_leds(self):
        """Update LED states based on timers"""
        current_time = self.robot.getTime() * 1000
        for led_index, start_time in self.led_timers.items():
            if start_time > 0 and current_time - start_time > self.LED_DURATION:
                self.leds[led_index].set(0)
                self.led_timers[led_index] = 0
                print(f"LED {led_index} turned OFF ({self.direction_names[led_index]} position)")

    def get_heading_angle(self):
        """Calculate current heading angle from compass"""
        north = self.compass.getValues()
        angle = degrees(atan2(north[1], north[0]))
        return (angle % 360) + 1

    def compass_bearing(self, current_coordinate, destination_coordinate):
        """Calculate bearing to destination"""
        theta_radians = math.atan2(destination_coordinate[1] - current_coordinate[1],
                                 destination_coordinate[0] - current_coordinate[0])
        theta_degrees = math.degrees(theta_radians)
        compass_bearing = (90 - theta_degrees + 360) % 360
        return compass_bearing

    def is_at_destination(self, current_coordinate):
        """Check if robot is at the destination within tolerance"""
        return (current_coordinate[0] > -0.04 and current_coordinate[0] < 0.04 and
                current_coordinate[1] > -0.04 and current_coordinate[1] < 0.04)

    def calculate_distance(self, current_coordinate, destination):
        """Calculate Euclidean distance to destination"""
        return math.sqrt((destination[0] - current_coordinate[0])**2 +
                        (destination[1] - current_coordinate[1])**2)

    def get_speed_for_distance(self, distance):
        """Calculate appropriate movement speed based on distance"""
        if distance < 0.1:
            return 1.0
        elif distance < 0.2:
            return 2.0
        elif distance < 0.3:
            return 3.0
        else:
            return 4.0

    def dance(self, current_time):
        """Execute dance sequence with timing"""
        SPIN_SPEED = 3
        dance_duration = 10
        current_duration = self.robot.getTime() - current_time
        
        if current_duration >= dance_duration:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            return False
            
        # Dance sequence timing
        if current_duration < 2:
            self.left_motor.setVelocity(SPIN_SPEED)
            self.right_motor.setVelocity(-SPIN_SPEED)
        elif current_duration < 4:
            self.left_motor.setVelocity(-SPIN_SPEED)
            self.right_motor.setVelocity(SPIN_SPEED)
        elif current_duration < 4.3:
            self.left_motor.setVelocity(2.0)
            self.right_motor.setVelocity(2.0)
        elif current_duration < 4.6:
            self.left_motor.setVelocity(-2.0)
            self.right_motor.setVelocity(-2.0)
        elif current_duration < 6:
            self.left_motor.setVelocity(-SPIN_SPEED)
            self.right_motor.setVelocity(SPIN_SPEED)
        elif current_duration < 6.3:
            self.left_motor.setVelocity(-2.0)
            self.right_motor.setVelocity(-2.0)
        elif current_duration < 6.6:
            self.left_motor.setVelocity(2.0)
            self.right_motor.setVelocity(2.0)
        elif current_duration < 8:
            self.left_motor.setVelocity(SPIN_SPEED)
            self.right_motor.setVelocity(-SPIN_SPEED)
        elif current_duration < 10:
            self.left_motor.setVelocity(SPIN_SPEED)
            self.right_motor.setVelocity(-SPIN_SPEED)
        
        return True

    def update(self):
        """Main update loop for petting mode"""
        collisions, has_interaction = self.check_collisions()
        self.update_leds()
        self.mood_check()
        
        # Update mood based on interactions
        current_mood = self.update_mood(has_interaction)
        
        gps_values = self.gps.getValues()
        current_coordinate = (gps_values[0], gps_values[1])
        bearing = self.compass_bearing(current_coordinate, self.destination)
        current_heading = self.get_heading_angle()
        
        # First priority: Get to destination if not there
        if not self.is_at_destination(current_coordinate):
            if self.dance_flag:
                # Store the fact that we were dancing and when we interrupted it
                if not self.dance_state_persistent:
                    self.dance_state_persistent = True
                    self.dance_interrupted_time = self.robot.getTime() - self.current_time
                    print("Dance interrupted, will resume later")
                self.dance_flag = False
            
            print("Moving to destination")
            # Calculate heading difference
            heading_diff = abs(current_heading - bearing)
            if heading_diff > 180:
                heading_diff = 360 - heading_diff
                
            # Adjust rotation speed based on how far we need to turn
            if heading_diff < 3:
                # We're facing the right direction, move forward
                distance = self.calculate_distance(current_coordinate, self.destination)
                speed = self.get_speed_for_distance(distance)
                self.left_motor.setVelocity(speed)
                self.right_motor.setVelocity(speed)
            elif heading_diff < 10:
                # Small adjustment needed
                turn_speed = self.SPIN_SPEED * 0.5
                self.left_motor.setVelocity(turn_speed)
                self.right_motor.setVelocity(-turn_speed)
            else:
                self.left_motor.setVelocity(self.SPIN_SPEED)
                self.right_motor.setVelocity(-self.SPIN_SPEED)
        
        # At destination: handle dancing based on mood
        else:
            odds = int(130 - current_mood)
            if not self.dance_flag:
                if self.dance_state_persistent:
                    # Resume interrupted dance
                    print("Resuming interrupted dance!")
                    self.current_time = self.robot.getTime() - self.dance_interrupted_time
                    self.dance_flag = True
                    self.dance_state_persistent = False
                # Randomly start dancing if mood is high enough and not resuming a dance
                elif current_mood > self.MOOD_THRESHOLD_HIGH and random.randint(1, odds) == 1:
                    print("Happy! Starting to dance!")
                    self.current_time = self.robot.getTime()
                    self.dance_flag = True
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
            else:
                # Continue dancing until sequence completes
                still_dancing = self.dance(self.current_time)
                if not still_dancing:
                    self.dance_flag = False
                    self.dance_state_persistent = False

    def enter(self):
        """Called when switching to petting mode"""
        print("Entering petting mode!")
        self.mood = 50  # Reset mood
        self.has_meowed = False  # Reset meow flag
        
    def exit(self):
        """Called when switching away from petting mode"""
        super().exit()  # Call parent exit to stop motors
        # Turn off all LEDs
        for led in self.leds:
            led.set(0)