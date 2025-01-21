#!/Users/whakimi/myenv/bin/python
# base_mode.py
class BaseMode:
    def __init__(self, robot):
        self.robot = robot
        self.TIME_STEP = 64
        self.initialize_devices()
        
    def initialize_devices(self):
        """Initialize any common devices used across modes"""
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        
        # Set motors to velocity mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
    def enter(self):
        """Called when this mode becomes active"""
        pass
        
    def exit(self):
        """Called when switching away from this mode"""
        # Stop motors
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
    def update(self):
        """Main update loop for the mode"""
        pass