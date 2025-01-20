from controller import Robot, Compass, GPS, LED, DistanceSensor
import math
import time
from math import atan2, degrees
import random

# Constants
TIME_STEP = 64  # Webots simulation time step (ms)
SPIN_SPEED = 2.0  # Speed for spinning
destination = (0, 0)
#FORWARD_SPEED = 2.0

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
        
def check_collisions():
    """Check for very close objects using distance sensors"""
    threshold = 1000  # Very high value indicates very close object
    collisions = []
        
    for i, sensor in enumerate(ps):
        value = sensor.getValue()
            # Print sensor values for debugging
            #print(f"Sensor {i} ({self.direction_names[i]}): {value}")
             
        if value > threshold:
            print(f"Very close object detected at {direction_names[i]}")
            collisions.append(i)
            blink_led(i)
        
    return collisions
  
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

def cartesian_calc_destination_theta_in_degrees(current_coordinate, destination_coordinate):

    delta_y = destination_coordinate[1] - current_coordinate[1]
    delta_x = destination_coordinate[0] - current_coordinate[0]
    return math.atan2(delta_y, delta_x) * 180 / math.pi


def cartesian_calc_theta_dot(heading, destination_theta):

    theta_dot = destination_theta - heading

    if theta_dot > 180:
        theta_dot = -(360 - theta_dot)
    elif theta_dot < -180:
        theta_dot = 360 + theta_dot

    return theta_dot


def cartesian_calc_rotated_theta_by_theta_dot(theta, theta_dot):

    if theta_dot == 0:
        return theta

    theta += theta_dot

    if theta < 0:
        theta = theta + 360
    elif theta >= 360:
        theta = theta - 360

    return theta

def compass_bearing(current_coordinate, destination_coordinate):

    theta_radians = math.atan2(destination_coordinate[1] - current_coordinate[1], destination_coordinate[0] - current_coordinate[0])
    theta_degrees = math.degrees(theta_radians)
    compass_bearing = (90 - theta_degrees + 360) % 360
    return compass_bearing
    
def dance(current_time):
    SPIN_SPEED = 3  # Speed for spinning
    if robot.getTime() - current_time < 2:
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
    elif robot.getTime() - current_time < 4:
        left_motor.setVelocity(-SPIN_SPEED)
        right_motor.setVelocity(SPIN_SPEED)
    elif robot.getTime() - current_time < 4.3:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
    elif robot.getTime() - current_time < 4.6:
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(-2.0)
    elif robot.getTime() - current_time < 6:
        left_motor.setVelocity(-SPIN_SPEED)
        right_motor.setVelocity(SPIN_SPEED)
    elif robot.getTime() - current_time < 6.3:
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(-2.0)
    elif robot.getTime() - current_time < 6.6:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
    elif robot.getTime() - current_time < 8:
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
    elif robot.getTime() - current_time < 10:
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
    elif robot.getTime() - current_time < 12:
        left_motor.setVelocity(-SPIN_SPEED)
        right_motor.setVelocity(SPIN_SPEED)
    elif robot.getTime() - current_time < 12.3:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
    elif robot.getTime() - current_time < 12.6:
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(-2.0)
    elif robot.getTime() - current_time < 14:
        left_motor.setVelocity(-SPIN_SPEED)
        right_motor.setVelocity(SPIN_SPEED)
    elif robot.getTime() - current_time < 14.3:
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(-2.0)
    elif robot.getTime() - current_time < 14.6:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
    elif robot.getTime() - current_time < 16:
        left_motor.setVelocity(SPIN_SPEED)
        right_motor.setVelocity(-SPIN_SPEED)
        
    # stop moving the robot and return spin speed to its normal value
    else:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        SPIN_SPEED = 1.0  
        return False 
    return True
 

    
# Set to True to start dancing -> skips the moving part until the dance finishes
dance_flag = False
current_time = 0

# Main loop
while robot.step(TIME_STEP) != -1:
    check_collisions()
    update_leds()
    
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    gps_values = gps.getValues()
    current_coordinate = (gps_values[0], gps_values[1])
    bearing = compass_bearing(current_coordinate, destination)
    
    current_heading = get_heading_angle()
    #print(f"Angle heading: {current_heading}")
    
    if not dance_flag:
    
        # randomly decides to dance whenever
        if random.randint(1, 100) == 50:
                print("start dancing")
                current_time = robot.getTime()
                dance_flag = True
                
        if(current_coordinate[0] > -0.05 and current_coordinate[0] < 0.05 and current_coordinate[1] > -0.05 and current_coordinate[1] < 0.05):
            print("is in circle")
             
        else:
            print(f"bearing: {bearing}")
            if (current_heading > bearing - 3 and current_heading < bearing + 3):
                #print("is in the right direction")
                left_motor.setVelocity(4.0)
                right_motor.setVelocity(4.0)
            else:
                #print("not in the right direction")
                left_motor.setVelocity(SPIN_SPEED)
                right_motor.setVelocity(-SPIN_SPEED)    
    else:
        # keeps dancing until the 16 second dance finishes
        dance_flag = dance(current_time)
        
 