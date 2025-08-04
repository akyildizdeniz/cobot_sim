#!/usr/bin/env python3

"""
Emergency Stop Monitor Node - Dedicated Safety Authority

This node aggregates emergency stop signals from multiple sources and provides
the authoritative emergency stop state for the entire system. It follows
industrial safety patterns where a central safety controller makes the final
emergency stop decisions.

Features:
- GUI input source (main testing interface)
- File-based interface for external systems and headless testing.
- Hardware interface (ready for future expansion)
- Safety-first logic: ANY active source triggers system stop. By design, 
if not all estops are cleared the system will not proceed as normal.
- Comprehensive logging of all safety events
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
import time


class EmergencyStopMonitor(Node):
    def __init__(self):
        super().__init__('emergency_stop_monitor')
        
        # Declare parameters
        self.declare_parameter('file_check_period', 0.1)
        self.declare_parameter('estop_file_path', '/tmp/estop.txt')
        
        # Emergency stop input states
        self.gui_estop = False
        self.hardware_estop = False  # Future expansion
        self.file_estop = False
        
        # Current authoritative state
        self.current_estop = False
        
        # Setup subscribers for e-stop sources
        # We do not have a hardware requirement now, but adding here to show how this system could be extended.
        self.create_subscription(Bool, 'estop_gui', self.gui_estop_callback, 10)
        self.create_subscription(Bool, 'estop_hardware', self.hardware_estop_callback, 10)
        
        # Authoritative emergency stop publisher
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)
        
        # File monitoring timer
        file_check_period = self.get_parameter('file_check_period').value
        self.create_timer(file_check_period, self.check_file_estop)
        
        # State update timer
        self.create_timer(0.05, self.update_estop_state)
        
        # Initialize file interface
        self.setup_file_interface()
        
        self.get_logger().info("Emergency Stop Monitor started - Safety Authority Active")
        self.print_status()

    def setup_file_interface(self):
        """Initialize the file-based emergency stop interface"""
        estop_file = self.get_parameter('estop_file_path').value
        
        try:
            # Create initial file with 'false' state
            with open(estop_file, 'w') as f:
                f.write('false\n')
            self.get_logger().info(f"File interface initialized: {estop_file}")
            self.get_logger().info(f"Usage: echo 'true' > {estop_file}  # Trigger")
            self.get_logger().info(f"       echo 'false' > {estop_file} # Clear")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize file interface: {e}")

    def print_status(self):
        """Print current emergency stop status"""
        print("\n" + "="*60)
        print("EMERGENCY STOP MONITOR - SAFETY AUTHORITY")
        print("="*60)
        print("Active Input Sources:")
        print(f"  GUI Button:     {'ACTIVE' if self.gui_estop else 'CLEAR'}")
        print(f"  File Interface: {'ACTIVE' if self.file_estop else 'CLEAR'}")
        print(f"  Hardware:       {'ACTIVE' if self.hardware_estop else 'CLEAR'} (ready for expansion)")
        print("-" * 60)
        print(f"SYSTEM STATUS:  {'EMERGENCY STOP' if self.current_estop else 'NORMAL OPERATION'}")
        print("="*60)

    def gui_estop_callback(self, msg):
        """Handle emergency stop from GUI buttons"""
        old_state = self.gui_estop
        self.gui_estop = msg.data
        
        if old_state != self.gui_estop:
            action = "triggered" if self.gui_estop else "cleared"
            self.get_logger().info(f"GUI emergency stop {action}")

    def hardware_estop_callback(self, msg):
        """Handle emergency stop from hardware input (future expansion)"""
        old_state = self.hardware_estop
        self.hardware_estop = msg.data
        
        if old_state != self.hardware_estop:
            action = "triggered" if self.hardware_estop else "cleared"
            self.get_logger().warn(f"Hardware emergency stop {action}")

    def check_file_estop(self):
        """Monitor file-based emergency stop interface"""
        estop_file = self.get_parameter('estop_file_path').value
        
        try:
            if os.path.exists(estop_file):
                with open(estop_file, 'r') as f:
                    content = f.read().strip().lower()
                
                old_state = self.file_estop
                self.file_estop = content in ['true', '1', 'yes', 'on']
                
                if old_state != self.file_estop:
                    action = "triggered" if self.file_estop else "cleared"
                    self.get_logger().info(f"File emergency stop {action}")
                    
        except Exception as e:
            self.get_logger().debug(f"File check error: {e}")

    def update_estop_state(self):
        """
        Aggregate all emergency stop inputs and publish authoritative state.
        
        Safety Logic:
        - Emergency stop is ACTIVE if ANY input source is active
        - Emergency stop is CLEAR only when ALL sources are clear
        - This implements fail-safe logic for safety systems
        """
        old_estop = self.current_estop
        
        # Safety-first aggregation: ANY active source triggers e-stop
        self.current_estop = (
            self.gui_estop or 
            self.hardware_estop or
            self.file_estop
        )
        
        # Publish state change
        if old_estop != self.current_estop:
            self.publish_estop_state()
            self.log_state_change(old_estop, self.current_estop)
            self.print_status()

    def publish_estop_state(self):
        """Publish the authoritative emergency stop state"""
        msg = Bool(data=self.current_estop)
        self.estop_pub.publish(msg)

    def log_state_change(self, old_state: bool, new_state: bool):
        """Log emergency stop state changes with source information"""
        if new_state:  # Emergency stop activated
            active_sources = []
            if self.gui_estop:
                active_sources.append("GUI")
            if self.hardware_estop:
                active_sources.append("Hardware")
            if self.file_estop:
                active_sources.append("File")
            
            sources_str = ", ".join(active_sources)
            self.get_logger().error(f"EMERGENCY STOP ACTIVATED - Sources: {sources_str}")
            
        else:  # Emergency stop cleared
            self.get_logger().info("Emergency stop CLEARED - All sources inactive")

    def get_system_status(self) -> dict:
        """Get comprehensive system status for diagnostics"""
        return {
            'authoritative_estop': self.current_estop,
            'input_sources': {
                'gui': self.gui_estop,
                'hardware': self.hardware_estop,
                'file': self.file_estop
            },
            'active_sources': [
                source for source, active in {
                    'gui': self.gui_estop,
                    'hardware': self.hardware_estop,
                    'file': self.file_estop
                }.items() if active
            ]
        }


def main():
    rclpy.init()
    
    try:
        monitor = EmergencyStopMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nShutting down Emergency Stop Monitor...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()