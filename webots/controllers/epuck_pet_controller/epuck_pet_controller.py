#!/Users/whakimi/myenv/bin/python
from controller import Robot, Camera, Compass, GPS, LED, DistanceSensor
import math
import time
from math import atan2, degrees
import random
import cv2
import numpy as np

# Constants
TIME_STEP = 64
SPIN_SPEED = 2.0
CENTER_POSITION = (0, 0)
FEEDING_POSITION = (1.0, 1.0)  # Example position for feeding spot
MOOD_THRESHOLD_HIGH = 55
MOOD_THRESHOLD_LOW = 25
MOOD_INCREASE = 5
MOOD_DECREASE = 0.5
LED_DURATION = 3000

# Robot modes
MODE_IDLE = 0
MODE_PETTING = 1
MODE_FEEDING = 2

# Feeding mode states
FEEDING_STATE_GOTO_MARKER = 0
FEEDING_STATE_CHARGING = 1
FEEDING_STATE_RETURN_CENTER = 2
FEEDING_STATE_COMPLETE = 3

# Direction mapping
direction_names = {
    0: "FRONT", 1: "FRONT-RIGHT", 2: "RIGHT", 3: "BACK-RIGHT",
    4: "BACK", 5: "BACK-LEFT", 6: "LEFT", 7: "FRONT-LEFT"
}

# Initialize robot and devices
robot = Robot()
compass = robot.getDevice("compass")
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Set up camera and ArUco detection
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Enable sensors
compass.enable(TIME_STEP)
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

# State variables
mood = 50
current_mode = MODE_IDLE
feeding_state = FEEDING_STATE_GOTO_MARKER
dance_flag = False
current_time = 0
led_timers = {i: 0 for i in range(len(leds))}
charging_complete = False
led_sequence_start_time = 0
marker_position = None

def check_aruco_markers():
    """Check camera for ArUco markers and their positions"""
    try:
        img = camera.getImage()
        if not img:
            return None, None
            
        img_array = np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
        gray = cv2.cvtColor(img_array, cv2.COLOR_BGRA2GRAY)
        
        cv2.imshow('Camera View', gray)
        cv2.waitKey(1)
        
        corners, ids, _ = detector.detectMarkers(gray)
        
        if ids is not None and len(corners) > 0:
            # Calculate marker position relative to robot
            marker_center = corners[0][0].mean(axis=0)
            # Convert pixel coordinates to rough world coordinates
            marker_x = (marker_center[0] - camera.getWidth()/2) * 0.01
            marker_y = (camera.getHeight() - marker_center[1]) * 0.01
            return ids.flatten()[0], (marker_x, marker_y)
        return None, None
                    
    except Exception as e:
        print(f"Warning: Error checking ArUco markers: {e}")
        return None, None

def check_collisions():
    """Check for nearby objects using distance sensors"""
    close_threshold = 1000
    near_threshold = 100
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
    else:
        mood = max(0, mood - MOOD_DECREASE)
    print(f"Current mood: {mood}")
    return mood

def get_heading_angle():
    """Get current heading angle from compass"""
    north = compass.getValues()
    angle = degrees(atan2(north[1], north[0]))
    return (angle % 360) + 1

def compass_bearing(current_pos, target_pos):
    """Calculate bearing to target position"""
    theta_radians = math.atan2(target_pos[1] - current_pos[1],
                              target_pos[0] - current_pos[0])
    theta_degrees = math.degrees(theta_radians)
    return (90 - theta_degrees + 360) % 360

def calculate_distance(pos1, pos2):
    """Calculate distance between two positions"""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def move_to_position(current_pos, target_pos):
    """Move robot towards target position, returns True if reached"""
    distance = calculate_distance(current_pos, target_pos)
    if distance < 0.04:  # At destination
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return True

    bearing = compass_bearing(current_pos, target_pos)
    current_heading = get_heading_angle()
    heading_diff = abs(current_heading - bearing)
    
    if heading_diff > 180:
        heading_diff = 360 - heading_diff

    if heading_diff < 3:
        # Move forward
        speed = min(4.0, max(1.0, distance * 10))
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)
    else:
        # Turn to face target
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
    
    return False

def blink_led(led_index):
    """Blink a specific LED"""
    if 0 <= led_index < len(leds):
        current_time = robot.getTime() * 1000
        leds[led_index].set(1)
        led_timers[led_index] = current_time

def update_leds():
    """Update LED states based on timers"""
    current_time = robot.getTime() * 1000
    for led_index, start_time in led_timers.items():
        if start_time > 0 and current_time - start_time > LED_DURATION:
            leds[led_index].set(0)
            led_timers[led_index] = 0

def feeding_mode_led_sequence():
    """LED sequence for feeding mode - gradual charging effect"""
    global charging_complete, led_sequence_start_time
    
    if led_sequence_start_time == 0:
        led_sequence_start_time = robot.getTime()
        for led in leds:  # Start with all LEDs off
            led.set(0)
    
    time_elapsed = robot.getTime() - led_sequence_start_time
    
    if time_elapsed > 8:  # Complete sequence after 8 seconds
        charging_complete = True
        return
    
    # Update LED every second
    led_to_light = int(time_elapsed)
    if led_to_light < len(leds):
        for i in range(led_to_light + 1):
            leds[i].set(1)

def handle_feeding_mode(current_pos):
    """Handle the feeding mode state machine"""
    global feeding_state, current_mode, charging_complete, led_sequence_start_time, marker_position
    
    if feeding_state == FEEDING_STATE_GOTO_MARKER:
        if marker_position is None:
            # Try to find the ArUco marker
            marker_id, marker_pos = check_aruco_markers()
            if marker_id == 2:  # Feeding marker
                marker_position = marker_pos
        
        if marker_position is not None:
            if move_to_position(current_pos, marker_position):
                feeding_state = FEEDING_STATE_CHARGING
                
    elif feeding_state == FEEDING_STATE_CHARGING:
        # Stop moving while charging
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        # Run LED charging sequence
        feeding_mode_led_sequence()
        
        if charging_complete:
            feeding_state = FEEDING_STATE_RETURN_CENTER
            
    elif feeding_state == FEEDING_STATE_RETURN_CENTER:
        if move_to_position(current_pos, CENTER_POSITION):
            feeding_state = FEEDING_STATE_COMPLETE
            
    elif feeding_state == FEEDING_STATE_COMPLETE:
        # Reset states and switch to idle mode
        current_mode = MODE_IDLE
        feeding_state = FEEDING_STATE_GOTO_MARKER
        charging_complete = False
        marker_position = None
        led_sequence_start_time = 0
        # Turn off all LEDs
        for led in leds:
            led.set(0)

def circular_movement():
    """Make the robot move in a circular pattern"""
    CIRCLE_SPEED = 2.0
    TURNING_RATIO = 0.7
    left_motor.setVelocity(CIRCLE_SPEED)
    right_motor.setVelocity(CIRCLE_SPEED * TURNING_RATIO)

def mood_check():
    """Check and update LED patterns based on mood"""
    if mood > MOOD_THRESHOLD_HIGH:
        for led in leds:
            if led.get() == 1:
                blinks_led(led, True)
            else:
                blinks_led(led, False)
    else:
        for led in leds:
            blinks_led(led, True)

def blinks_led(led, isON):
    """Control LED state"""
    if led is not None:
        if isON:
            led.set(0)
        else:
            led.set(1)

# Main loop
print("Robot starting...")
while robot.step(TIME_STEP) != -1:
    # Get current position
    gps_values = gps.getValues()
    current_pos = (gps_values[0], gps_values[1])
    
    # Check for mode changes via ArUco markers
    marker_id, marker_pos = check_aruco_markers()
    if marker_id is not None:
        if marker_id == 1:
            if current_mode != MODE_PETTING:
                current_mode = MODE_PETTING
                print("Switching to petting mode")
        elif marker_id == 2:
            if current_mode != MODE_FEEDING:
                current_mode = MODE_FEEDING
                feeding_state = FEEDING_STATE_GOTO_MARKER
                marker_position = marker_pos
                print("Switching to feeding mode")
        elif marker_id == 3:
            if current_mode != MODE_IDLE:
                current_mode = MODE_IDLE
                print("Switching to idle mode")
    
    # Basic behaviors that always run
    collisions, has_interaction = check_collisions()
    update_leds()
    current_mood = update_mood(has_interaction)
    
    # Handle different modes
    if current_mode == MODE_PETTING:
        if has_interaction:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
        else:
            left_motor.setVelocity(SPIN_SPEED)
            right_motor.setVelocity(-SPIN_SPEED)
        mood_check()
            
    elif current_mode == MODE_FEEDING:
        handle_feeding_mode(current_pos)
            
    else:  # MODE_IDLE
        circular_movement()
        mood_check()
