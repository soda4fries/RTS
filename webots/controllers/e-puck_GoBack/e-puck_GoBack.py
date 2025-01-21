#!/Users/whakimi/myenv/bin/python
# main_controller.py
from controller import Robot
from petting_mode import PettingMode
from feeding_mode import FeedingMode
from obstacle_mode import ObstacleMode
from follow_mode import FollowMode
import tkinter as tk
from tkinter import ttk
import sys

class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Robot Controller")
        self.root.geometry("300x400")
        
        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Robot Mode Controller", font=('Helvetica', 14, 'bold'))
        title_label.grid(row=0, column=0, pady=10)
        
        # Current mode display
        self.mode_var = tk.StringVar(value="Current Mode: None")
        mode_label = ttk.Label(main_frame, textvariable=self.mode_var, font=('Helvetica', 12))
        mode_label.grid(row=1, column=0, pady=10)
        
        # Mode buttons
        ttk.Label(main_frame, text="Select Mode:", font=('Helvetica', 12)).grid(row=2, column=0, pady=5)
        
        # Create a frame for buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, pady=10)
        
        # Mode buttons with descriptions
        modes = [
            ("Petting Mode", 'P', "Interact with the robot through petting"),
            ("Feeding Mode", 'F', "Feed and maintain robot's hunger"),
            ("Obstacle Mode", 'O', "Navigate and avoid obstacles"),
            ("Follow Mode", 'L', "Follow detected objects")
        ]
        
        for i, (mode_name, mode_key, description) in enumerate(modes):
            mode_frame = ttk.Frame(button_frame)
            mode_frame.grid(row=i, column=0, pady=5, sticky=tk.W)
            
            btn = ttk.Button(mode_frame, 
                           text=mode_name,
                           command=lambda k=mode_key: self.switch_mode(k))
            btn.grid(row=0, column=0, padx=5)
            
            desc_label = ttk.Label(mode_frame, text=description, font=('Helvetica', 10))
            desc_label.grid(row=0, column=1, padx=5)
        
        # Status display
        self.status_var = tk.StringVar(value="Status: Ready")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, font=('Helvetica', 10))
        status_label.grid(row=4, column=0, pady=10)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def switch_mode(self, mode_key):
        """Switch robot mode and update GUI"""
        try:
            self.controller.switch_mode(mode_key)
            self.mode_var.set(f"Current Mode: {self.controller.current_mode.__class__.__name__}")
            self.status_var.set(f"Status: Switched to {mode_key} mode")
        except Exception as e:
            self.status_var.set(f"Error: {str(e)}")
    
    def update(self):
        """Update GUI and process robot controller"""
        self.root.update()
        
    def on_closing(self):
        """Handle window closing"""
        if self.controller.current_mode:
            self.controller.current_mode.exit()
        self.root.destroy()
        sys.exit()

class MainController:
    def __init__(self):
        self.robot = Robot()
        self.TIME_STEP = 64
        
        # Initialize all modes
        self.modes = {
            'P': PettingMode(self.robot),
            'F': FeedingMode(self.robot),
            'O': ObstacleMode(self.robot),
            'L': FollowMode(self.robot)
        }
        self.current_mode = None
        
        # Initialize motors for idle state
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Create GUI
        self.gui = RobotGUI(self)
            
    def switch_mode(self, mode_key):
        """Switch to specified mode"""
        if mode_key in self.modes:
            if self.current_mode:
                print(f"Exiting {self.current_mode.__class__.__name__}")
                self.current_mode.exit()
            self.current_mode = self.modes[mode_key]
            print(f"Entering {self.current_mode.__class__.__name__}")
            self.current_mode.enter()
                
    def run(self):
        """Main control loop"""
        while self.robot.step(self.TIME_STEP) != -1:
            try:
                self.gui.update()
                if self.current_mode:
                    self.current_mode.update()
                else:
                    # Idle behavior
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
            except tk.TclError:  # GUI was closed
                break

if __name__ == "__main__":
    controller = MainController()
    controller.run()