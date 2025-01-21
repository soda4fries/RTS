# #!/Users/whakimi/myenv/bin/python
# import cv2
# import numpy as np
# import tkinter as tk
# from tkinter import ttk
# from PIL import Image, ImageTk

# class CameraWindow:
#     def __init__(self):
#         # Create main window
#         self.window = tk.Tk()
#         self.window.title("Robot Camera View")
#         self.window.geometry("800x600")
        
#         # Create frames
#         self.camera_frame = ttk.Frame(self.window)
#         self.camera_frame.pack(pady=10)
        
#         self.control_frame = ttk.Frame(self.window)
#         self.control_frame.pack(pady=5)
        
#         # Camera view
#         self.camera_label = ttk.Label(self.camera_frame)
#         self.camera_label.pack()
        
#         # Controls for HSV thresholds
#         # Hue
#         ttk.Label(self.control_frame, text="Hue Min:").grid(row=0, column=0, padx=5)
#         self.hue_min_var = tk.IntVar(value=0)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=179,
#             orient=tk.HORIZONTAL, 
#             variable=self.hue_min_var,
#             length=200
#         ).grid(row=0, column=1, padx=5)
        
#         ttk.Label(self.control_frame, text="Hue Max:").grid(row=0, column=2, padx=5)
#         self.hue_max_var = tk.IntVar(value=179)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=179,
#             orient=tk.HORIZONTAL, 
#             variable=self.hue_max_var,
#             length=200
#         ).grid(row=0, column=3, padx=5)
        
#         # Saturation
#         ttk.Label(self.control_frame, text="Sat Min:").grid(row=1, column=0, padx=5)
#         self.sat_min_var = tk.IntVar(value=0)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=255,
#             orient=tk.HORIZONTAL, 
#             variable=self.sat_min_var,
#             length=200
#         ).grid(row=1, column=1, padx=5)
        
#         ttk.Label(self.control_frame, text="Sat Max:").grid(row=1, column=2, padx=5)
#         self.sat_max_var = tk.IntVar(value=30)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=255,
#             orient=tk.HORIZONTAL, 
#             variable=self.sat_max_var,
#             length=200
#         ).grid(row=1, column=3, padx=5)
        
#         # Value
#         ttk.Label(self.control_frame, text="Val Min:").grid(row=2, column=0, padx=5)
#         self.val_min_var = tk.IntVar(value=200)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=255,
#             orient=tk.HORIZONTAL, 
#             variable=self.val_min_var,
#             length=200
#         ).grid(row=2, column=1, padx=5)
        
#         ttk.Label(self.control_frame, text="Val Max:").grid(row=2, column=2, padx=5)
#         self.val_max_var = tk.IntVar(value=255)
#         ttk.Scale(
#             self.control_frame, 
#             from_=0, 
#             to=255,
#             orient=tk.HORIZONTAL, 
#             variable=self.val_max_var,
#             length=200
#         ).grid(row=2, column=3, padx=5)
        
#         # View mode selection
#         self.view_mode = tk.StringVar(value="normal")
#         ttk.Radiobutton(
#             self.control_frame, 
#             text="Normal View", 
#             variable=self.view_mode, 
#             value="normal"
#         ).grid(row=3, column=0)
#         ttk.Radiobutton(
#             self.control_frame, 
#             text="Mask View", 
#             variable=self.view_mode, 
#             value="mask"
#         ).grid(row=3, column=1)
        
#         # Status display
#         self.status_var = tk.StringVar(value="Status: Searching for ball...")
#         self.status_label = ttk.Label(self.window, textvariable=self.status_var)
#         self.status_label.pack(pady=5)
        
#         # Ball detection parameters
#         self.min_ball_radius = 5
#         self.max_ball_radius = 50
        
#     def detect_ball(self, img_data, width, height):
#         """Process image and detect white ball using HSV"""
#         if img_data is None:
#             return False, None, None
            
#         # Convert to numpy array and reshape
#         img_array = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
#         img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
        
#         # Convert to HSV
#         hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
#         # Get current HSV thresholds
#         lower_white = np.array([
#             self.hue_min_var.get(),
#             self.sat_min_var.get(),
#             self.val_min_var.get()
#         ])
#         upper_white = np.array([
#             self.hue_max_var.get(),
#             self.sat_max_var.get(),
#             self.val_max_var.get()
#         ])
        
#         # Create mask
#         mask = cv2.inRange(hsv, lower_white, upper_white)
        
#         # Clean up mask
#         kernel = np.ones((5,5), np.uint8)
#         mask = cv2.erode(mask, kernel, iterations=1)
#         mask = cv2.dilate(mask, kernel, iterations=2)
        
#         # Find contours
#         contours, _ = cv2.findContours(
#             mask, 
#             cv2.RETR_EXTERNAL, 
#             cv2.CHAIN_APPROX_SIMPLE
#         )
        
#         # Initialize detection variables
#         ball_detected = False
#         ball_position = None
#         max_area = 0
#         best_circle = None
        
#         # Process each contour
#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area > max_area:
#                 # Find enclosing circle
#                 (x, y), radius = cv2.minEnclosingCircle(contour)
                
#                 if self.min_ball_radius < radius < self.max_ball_radius:
#                     # Calculate circularity
#                     circularity = 4 * np.pi * area / (2 * np.pi * radius) ** 2
                    
#                     if circularity > 0.7:  # Relaxed circularity check
#                         max_area = area
#                         best_circle = (int(x), int(y), int(radius))
#                         # Convert to rough world coordinates
#                         ball_x = (x - width/2) * 0.01
#                         ball_y = (height - y) * 0.01
#                         ball_position = (ball_x, ball_y)
#                         ball_detected = True
        
#         # Draw detection visualization
#         vis_img = img_bgr.copy()
#         if ball_detected and best_circle is not None:
#             x, y, radius = best_circle
#             # Draw circle around ball
#             cv2.circle(vis_img, (x, y), radius, (0, 255, 0), 2)
#             cv2.circle(vis_img, (x, y), 2, (0, 0, 255), 3)
#             self.status_var.set(f"Status: Ball detected at ({x}, {y})")
#         else:
#             self.status_var.set("Status: Searching for ball...")
        
#         # Update display
#         display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if self.view_mode.get() == "mask" else vis_img
#         self.update_image(display_img)
        
#         return ball_detected, ball_position, vis_img
        
#     def update_image(self, image):
#         """Update the camera view with new image"""
#         if image is not None:
#             # Convert to RGB for tkinter
#             image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#             # Convert to PhotoImage
#             image = Image.fromarray(image)
#             photo = ImageTk.PhotoImage(image=image)
#             self.camera_label.configure(image=photo)
#             self.camera_label.image = photo  # Keep a reference
            
#     def update(self):
#         """Update the window"""
#         try:
#             self.window.update()
#         except tk.TclError:
#             pass  # Window was closed
            
#     def close(self):
#         """Clean up window"""
#         try:
#             self.window.destroy()
#         except:
#             pass