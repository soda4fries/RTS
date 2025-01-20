# main_controller.py
from controller import Robot, Keyboard
from petting_mode import PettingMode
from feeding_mode import FeedingMode
from obstacle_mode import ObstacleMode
from follow_mode import FollowMode

class MainController:
    def __init__(self):
        self.robot = Robot()
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(64)
        
        # Initialize all modes
        self.modes = {
            'P': PettingMode(self.robot),
            'F': FeedingMode(self.robot),
            'O': ObstacleMode(self.robot),
            'L': FollowMode(self.robot)
        }
        self.current_mode = None
        self.TIME_STEP = 64
        
        # Start with petting mode
        self.switch_mode('P')
        
    def switch_mode(self, mode_key):
        if mode_key in self.modes:
            if self.current_mode:
                print(f"Exiting {self.current_mode.__class__.__name__}")
                self.current_mode.exit()
            self.current_mode = self.modes[mode_key]
            print(f"Entering {self.current_mode.__class__.__name__}")
            self.current_mode.enter()
            
    def handle_input(self):
        key = self.keyboard.getKey()
        if key != -1:
            key_char = chr(key).upper()
            if key_char in self.modes:
                self.switch_mode(key_char)
                
    def run(self):
        while self.robot.step(self.TIME_STEP) != -1:
            self.handle_input()
            if self.current_mode:
                self.current_mode.update()

if __name__ == "__main__":
    controller = MainController()
    controller.run()