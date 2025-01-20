from controller import Robot, Compass, GPS, LED, DistanceSensor
import math
import time
from math import atan2, degrees
import random

# Constants
TIME_STEP = 64  # Webots simulation time step (ms)
SPIN_SPEED = 2.0  # Speed for spinning
destination = (0, 0)
MOOD_THRESHOLD_HIGH = 65  # Threshold for high mood (easier to reach)
MOOD_THRESHOLD_LOW = 25   # Threshold for low mood
MOOD_INCREASE = 5         # How much mood increases per interaction
MOOD_DECREASE = 0.5       # How much mood decreases per no interaction (slower decay)

# Initialize robot and devices
robot = Robot()
compass = robot.getDevice("compass")
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Enable sensors
compass.enable(TIME_STEP)

# Get the GPS device
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# Set motors to velocity mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
        
# Initialize LEDs
leds = []
for i in range(8):
    led = robot.getDevice(f'led{i}')
    if led:
        leds.append(led)
        
# Initialize proximity sensors
ps = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    if sensor:
        sensor.enable(TIME_STEP)
        ps.append(sensor)
        
# LED timer dictionary
led_timers = {i: 0 for i in range(len(leds))}
LED_DURATION = 3000  # 3 seconds in milliseconds

# Direction mapping for better debug messages
direction_names = {
    0: "FRONT",
    1: "FRONT-RIGHT",
    2: "RIGHT",
    3: "BACK-RIGHT",
    4: "BACK",
    5: "BACK-LEFT",
    6: "LEFT",
    7: "FRONT-LEFT"
}

# Mood system
mood = 50  # Start with neutral mood
last_interaction_time = 0
        
def check_collisions():
    """Check for nearby objects using distance sensors"""
    close_threshold = 1000  # Threshold for very close objects
    near_threshold = 100   # Threshold for nearby objects - will trigger interaction
    collisions = []
    has_interaction = False
        
    for i, sensor in enumerate(ps):
        value = sensor.getValue()
        if value > close_threshold:
            print(f"Very close object detected at {direction_names[i]}")
            collisions.append(i)
            blink_led(i)
            has_interaction = True
        elif value > near_threshold:
            print(f"Nearby object detected at {direction_names[i]}")
            has_interaction = True
        
    return collisions, has_interaction

def update_mood(has_interaction):
    """Update mood based on interactions"""
    global mood
    
    if has_interaction:
        mood = min(100, mood + MOOD_INCREASE)
        # Blink happy LEDs (front LEDs) when mood is high
        if mood > MOOD_THRESHOLD_HIGH:
            blink_led(0)
            blink_led(7)
    else:
        mood = max(0, mood - MOOD_DECREASE)
        # Blink sad LEDs (back LEDs) when mood is low
        if mood < MOOD_THRESHOLD_LOW:
            blink_led(3)
            blink_led(4)
    
    print(f"Current mood: {mood}")
    return mood
  
def blink_led(led_index):
    """Blink a specific LED"""
    if 0 <= led_index and led_index < len(leds):
        current_time = robot.getTime() * 1000
        leds[led_index].set(1)
        led_timers[led_index] = current_time
        print(f"LED {led_index} turned ON ({direction_names[led_index]} position)")

def update_leds():
    """Turn off LEDs after duration"""
    current_time = robot.getTime() * 1000
    for led_index, start_time in led_timers.items():
        if start_time > 0 and current_time - start_time > LED_DURATION:
            leds[led_index].set(0)
            led_timers[led_index] = 0
            print(f"LED {led_index} turned OFF ({direction_names[led_index]} position)")

def get_heading_angle():
    north = compass.getValues()  
    angle = degrees(atan2(north[1], north[0]))  
    return (angle % 360) + 1  

def compass_bearing(current_coordinate, destination_coordinate):
    theta_radians = math.atan2(destination_coordinate[1] - current_coordinate[1], 
                              destination_coordinate[0] - current_coordinate[0])
    theta_degrees = math.degrees(theta_radians)
    compass_bearing = (90 - theta_degrees + 360) % 360
    return compass_bearing

def is_at_destination(current_coordinate):
    """Check if robot is at the destination"""
    return (current_coordinate[0] > -0.04 and current_coordinate[0] < 0.04 and 
            current_coordinate[1] > -0.04 and current_coordinate[1] < 0.04)

def calculate_distance(current_coordinate, destination):
    """Calculate distance to destination"""
    return math.sqrt((destination[0] - current_coordinate[0])**2 + 
                    (destination[1] - current_coordinate[1])**2)

def get_speed_for_distance(distance):
    """Calculate appropriate speed based on distance to destination"""
    if distance < 0.1:  # Very close
        return 1.0
    elif distance < 0.2:  # Close
        return 2.0
    elif distance < 0.3:  # Medium distance
        return 3.0
    else:  # Far
        return 4.0
    
def dance(current_time):
    """Execute dance sequence"""
    SPIN_SPEED = 3  # Speed for spinning
    dance_duration = 16  # Total duration of dance in seconds
    current_duration = robot.getTime() - current_time
    
    if current_duration >= dance_duration:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return False
        
    # Dance sequence timing
    if current_duration < 2:
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
    elif current_duration < 4:
        left_motor.setVelocity(-SPIN_SPEED)
        right_motor.setVelocity(SPIN_SPEED)
    elif current_duration < 4.3:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
    # ... (rest of the dance sequence remains the same)
    
    return True

# Main control variables
dance_flag = False
dance_state_persistent = False  # Maintains dance state even when interrupted
current_time = 0
dance_interrupted_time = 0  # Stores the time when dance was interrupted

# Main loop
while robot.step(TIME_STEP) != -1:
    collisions, has_interaction = check_collisions()
    update_leds()
    
    # Update mood based on interactions
    current_mood = update_mood(has_interaction)
    
    gps_values = gps.getValues()
    current_coordinate = (gps_values[0], gps_values[1])
    bearing = compass_bearing(current_coordinate, destination)
    current_heading = get_heading_angle()
    
    # First priority: Get to destination if not there
    if not is_at_destination(current_coordinate):
        if dance_flag:
            # Store the fact that we were dancing and when we interrupted it
            if not dance_state_persistent:
                dance_state_persistent = True
                dance_interrupted_time = robot.getTime() - current_time
                print("Dance interrupted, will resume later")
            dance_flag = False  # Temporarily stop dancing to return to center
        
        print("Moving to destination")
        # Calculate heading difference
        heading_diff = abs(current_heading - bearing)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
            
        # Adjust rotation speed based on how far we need to turn
        if heading_diff < 3:
            # We're facing the right direction, move forward
            distance = calculate_distance(current_coordinate, destination)
            speed = get_speed_for_distance(distance)
            left_motor.setVelocity(speed)
            right_motor.setVelocity(speed)
        elif heading_diff < 10:
            # Small adjustment needed
            turn_speed = SPIN_SPEED * 0.5
            left_motor.setVelocity(turn_speed)
            right_motor.setVelocity(-turn_speed)
        else:
            left_motor.setVelocity(SPIN_SPEED)
            right_motor.setVelocity(-SPIN_SPEED)
    
    # At destination: handle dancing based on mood
    else:
        if not dance_flag:
            if dance_state_persistent:
                # Resume interrupted dance
                print("Resuming interrupted dance!")
                current_time = robot.getTime() - dance_interrupted_time
                dance_flag = True
                dance_state_persistent = False
            # Randomly start dancing if mood is high enough and not resuming a dance
            elif current_mood > MOOD_THRESHOLD_HIGH and random.randint(1, 50) == 1:
                print("Happy! Starting to dance!")
                current_time = robot.getTime()
                dance_flag = True
            else:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
        else:
            # Continue dancing until sequence completes
            still_dancing = dance(current_time)
            if not still_dancing:
                dance_flag = False
                dance_state_persistent = False  # Reset persistent state when dance completes