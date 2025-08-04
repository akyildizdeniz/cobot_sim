#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
from cobot_sim.msg import SpeedState
import tkinter as tk
from tkinter import ttk
import threading
import time

class Visualizer(Node):
    def __init__(self):
        super().__init__('system_visualizer_gui')

        # Data storage
        self.proximity_distance = 0.0
        self.estop_status = False
        self.speed_state = SpeedState.UNKNOWN
        self.joint_velocities = [0.0] * 6
        self.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']

        # ROS2 subscribers
        self.create_subscription(Float32, 'proximity', self.proximity_callback, 10)
        self.create_subscription(Bool, 'estop', self.estop_callback, 10)
        self.create_subscription(SpeedState, 'speed_state', self.speed_state_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # ROS2 publisher for e-stop
        self.estop_gui_pub = self.create_publisher(Bool, 'estop_gui', 10)

        # Initialize GUI
        self.setup_gui()
        self.update_gui()
        self.get_logger().info("System Visualizer GUI started")

    def proximity_callback(self, msg):
        self.proximity_distance = msg.data

    def estop_callback(self, msg):
        self.estop_status = msg.data

    def speed_state_callback(self, msg):
        self.speed_state = msg.current_state

    def joint_state_callback(self, msg):
        if len(msg.velocity) >= 6:
            self.joint_velocities = list(msg.velocity[:6])

    def setup_gui(self):
        """Create the GUI interface"""
        self.root = tk.Tk()
        self.root.title("Cobot Safety System Monitor")
        self.root.geometry("800x600")
        self.root.configure(bg='#2c3e50')
        # Title
        title_label = tk.Label(
            self.root, 
            text="Proximity-Based Speed Control System", 
            font=('Arial', 20, 'bold'),
            fg='white', 
            bg='#2c3e50'
        )
        title_label.pack(pady=20)
        # Main frame
        main_frame = tk.Frame(self.root, bg='#2c3e50')
        main_frame.pack(fill='both', expand=True, padx=20, pady=10)
        # Create sections
        self.create_proximity_section(main_frame)
        self.create_speed_control_section(main_frame)
        self.create_estop_section(main_frame)
        self.create_velocity_section(main_frame)
        # Status bar
        self.status_label = tk.Label(
            self.root,
            text="System Running",
            font=('Arial', 10),
            fg='white',
            bg='#34495e',
            relief='sunken'
        )
        self.status_label.pack(side='bottom', fill='x')

    def create_proximity_section(self, parent):
        """Create proximity sensor display"""
        frame = tk.LabelFrame(
            parent, 
            text="Proximity Sensor", 
            font=('Arial', 14, 'bold'),
            fg='white', 
            bg='#34495e',
            labelanchor='n'
        )
        frame.pack(fill='x', pady=10)
        # Distance display
        self.distance_label = tk.Label(
            frame,
            text="Distance: 0 mm",
            font=('Arial', 16, 'bold'),
            fg='#3498db',
            bg='#34495e'
        )
        self.distance_label.pack(pady=10)
        # Progress bar for distance
        self.distance_progress = ttk.Progressbar(
            frame,
            length=400,
            mode='determinate',
            maximum=1200
        )
        self.distance_progress.pack(pady=5)
        # Zone indicators
        zones_frame = tk.Frame(frame, bg='#34495e')
        zones_frame.pack(pady=5)
        tk.Label(zones_frame, text="STOP\n< 400mm", fg='#e74c3c', bg='#34495e', font=('Arial', 10, 'bold')).pack(side='left', padx=20)
        tk.Label(zones_frame, text="SLOW\n400-800mm", fg='#f39c12', bg='#34495e', font=('Arial', 10, 'bold')).pack(side='left', padx=20)
        tk.Label(zones_frame, text="FULL\n> 800mm", fg='#27ae60', bg='#34495e', font=('Arial', 10, 'bold')).pack(side='left', padx=20)
    
    def create_speed_control_section(self, parent):
        """Create speed control status display"""
        frame = tk.LabelFrame(
            parent,
            text="Speed Control State",
            font=('Arial', 14, 'bold'),
            fg='white',
            bg='#34495e',
            labelanchor='n'
        )
        frame.pack(fill='x', pady=10)
        self.speed_state_label = tk.Label(
            frame,
            text="UNKNOWN",
            font=('Arial', 24, 'bold'),
            fg='#95a5a6',
            bg='#34495e'
        )
        self.speed_state_label.pack(pady=20)
    
    def create_estop_section(self, parent):
        """Create emergency stop status display"""
        frame = tk.LabelFrame(
            parent,
            text="Emergency Stop",
            font=('Arial', 14, 'bold'),
            fg='white',
            bg='#34495e',
            labelanchor='n'
        )
        frame.pack(fill='x', pady=10)
        self.estop_indicator = tk.Label(
            frame,
            text="NORMAL",
            font=('Arial', 20, 'bold'),
            fg='#27ae60',
            bg='#34495e'
        )
        self.estop_indicator.pack(pady=10)
        # E-stop control buttons
        button_frame = tk.Frame(frame, bg='#34495e')
        button_frame.pack(pady=10)
        self.estop_button = tk.Button(
            button_frame,
            text="🚨 TRIGGER E-STOP",
            font=('Arial', 12, 'bold'),
            fg='white',
            bg='#e74c3c',
            activebackground='#c0392b',
            command=self.trigger_estop,
            width=15
        )
        self.estop_button.pack(side='left', padx=5)
        self.clear_button = tk.Button(
            button_frame,
            text="✅ CLEAR E-STOP",
            font=('Arial', 12, 'bold'),
            fg='white',
            bg='#27ae60',
            activebackground='#229954',
            command=self.clear_estop,
            width=15
        )
        self.clear_button.pack(side='left', padx=5)

    def trigger_estop(self):
        """Trigger emergency stop via button"""
        msg = Bool(data=True)
        self.estop_gui_pub.publish(msg)
        self.get_logger().info("GUI: Emergency stop triggered")

    def clear_estop(self):
        """Clear emergency stop via button"""
        msg = Bool(data=False)
        self.estop_gui_pub.publish(msg)
        self.get_logger().info("GUI: Emergency stop cleared")

    def create_velocity_section(self, parent):
        """Create joint velocity display"""
        frame = tk.LabelFrame(
            parent,
            text="Joint Velocities (rad/s)",
            font=('Arial', 14, 'bold'),
            fg='white',
            bg='#34495e',
            labelanchor='n'
        )
        frame.pack(fill='both', expand=True, pady=10)
        # Create velocity displays for each joint
        self.velocity_labels = []
        velocity_frame = tk.Frame(frame, bg='#34495e')
        velocity_frame.pack(fill='both', expand=True, padx=10, pady=10)

        for i, joint_name in enumerate(self.joint_names):
            joint_frame = tk.Frame(velocity_frame, bg='#34495e')
            joint_frame.pack(fill='x', pady=2)
            
            name_label = tk.Label(
                joint_frame,
                text=f"{joint_name}:",
                font=('Arial', 10),
                fg='white',
                bg='#34495e',
                width=15,
                anchor='w'
            )
            name_label.pack(side='left')
            velocity_label = tk.Label(
                joint_frame,
                text="0.000",
                font=('Arial', 10, 'bold'),
                fg='#3498db',
                bg='#34495e',
                width=10,
                anchor='e'
            )
            velocity_label.pack(side='right')
            self.velocity_labels.append(velocity_label)

    def update_gui(self):
        """Update GUI elements with current data"""
        try:
            # Update proximity display
            self.distance_label.config(text=f"Distance: {self.proximity_distance:.1f} mm")
            self.distance_progress['value'] = min(self.proximity_distance, 1200)
            # Update speed state
            state_colors = {
                SpeedState.FULL: '#27ae60',
                SpeedState.SLOW: '#f39c12', 
                SpeedState.STOP: '#e74c3c',
                SpeedState.UNKNOWN: '#95a5a6'
            }
            state_names = {
                SpeedState.FULL: 'FULL SPEED',
                SpeedState.SLOW: 'SLOW',
                SpeedState.STOP: 'STOP',
                SpeedState.UNKNOWN: 'UNKNOWN'
            }
            state_name = state_names.get(self.speed_state, 'UNKNOWN')
            state_color = state_colors.get(self.speed_state, '#95a5a6')
            self.speed_state_label.config(text=state_name, fg=state_color)
            # Update emergency stop status
            if self.estop_status:
                self.estop_indicator.config(text="EMERGENCY STOP", fg='#e74c3c')
                self.root.configure(bg='#8b0000')  # Dark red background
            else:
                self.estop_indicator.config(text="NORMAL", fg='#27ae60')
                self.root.configure(bg='#2c3e50')  # Normal background
            # Update joint velocities
            for i, velocity_label in enumerate(self.velocity_labels):
                if i < len(self.joint_velocities):
                    velocity = self.joint_velocities[i]
                    velocity_label.config(text=f"{velocity:.3f}")
                    # Color code based on velocity magnitude
                    abs_vel = abs(velocity)
                    if abs_vel > 0.1:
                        velocity_label.config(fg='#e74c3c')  # Red for high velocity
                    elif abs_vel > 0.01:
                        velocity_label.config(fg='#f39c12')  # Orange for medium velocity
                    else:
                        velocity_label.config(fg='#3498db')  # Blue for low/zero velocity
            # Update status
            current_time = time.strftime("%H:%M:%S")
            status_text = f"Last updated: {current_time} | Distance: {self.proximity_distance:.1f}mm | State: {state_name}"
            if self.estop_status:
                status_text += " | ⚠️ EMERGENCY STOP ACTIVE"
            self.status_label.config(text=status_text)
        except Exception as e:
            self.get_logger().error(f"Error updating GUI: {e}")
        # Schedule next update
        self.root.after(50, self.update_gui)  # Update every 50ms

    def run(self):
        """Run the GUI main loop"""
        self.root.mainloop()

def main():
    rclpy.init()
    # Create node
    visualizer = Visualizer()
    # Run ROS2 spinning in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(visualizer), daemon=True)
    ros_thread.start()
    try:
        # Run GUI main loop
        visualizer.run()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()