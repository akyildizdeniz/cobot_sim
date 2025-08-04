#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from cobot_sim.msg import SpeedState
import csv, time

class StateLogger(Node):
    def __init__(self):
        super().__init__('state_logger')

        # Initialize state variables
        self.estop = None
        self.speed_state = None

        # Setup CSV file with header
        self.file = open('speed_log.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'event_type', 'speed_state', 'estop'])  # Fixed header
        self.create_subscription(Bool, 'estop', self.cb_estop, 50)
        self.create_subscription(SpeedState, 'speed_state', self.cb_speed_state, 50)

        self.get_logger().info("State Logger started - logging to speed_log.csv")

    def cb_estop(self, msg):
        """Callback for logging estop state change"""
        old_estop = self.estop
        self.estop = msg.data

        # Log EVERY estop change
        if old_estop != self.estop:
            event_type = 'estop_triggered' if self.estop else 'estop_cleared'
            self.writer.writerow([
                time.time(), 
                event_type,
                self.speed_state,
                self.estop
            ])
            self.file.flush()
            self.get_logger().info(f"Logged estop change: {self.estop}")

    def cb_speed_state(self, msg):
        """Callback for logging speed state change"""
        old_speed = self.speed_state
        self.speed_state = msg.current_state

        # Log EVERY speed state change
        if old_speed != self.speed_state:
            self.writer.writerow([
                time.time(),
                'speed_change',
                self.speed_state,
                self.estop
            ])
            self.file.flush()
            self.get_logger().info(f"Logged speed change: {self.speed_state}")

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'file'):
            self.file.close()

def main():
    rclpy.init()
    node = StateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()