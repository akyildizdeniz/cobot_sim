#!/usr/bin/env python3

"""
Proximity Sensor Node

Simulates a proximity sensor that publishes distance readings to objects.
Generates realistic sensor data with configurable parameters for testing
the robot control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class ProximitySensor(Node):
    def __init__(self):
        super().__init__('proximity_sensor')
        
        # Declare parameters
        self.declare_parameter('distance_min', 200.0)
        self.declare_parameter('distance_max', 1200.0)
        self.declare_parameter('publish_rate', 0.05)
        self.declare_parameter('update_rate', 0.1)
        # hold duration represents how fast the sensor readings should change. here
        # to make it more realistic using a random time between 1-3 seconds.
        self.declare_parameter('hold_duration_min', 1.0)
        self.declare_parameter('hold_duration_max', 3.0)

        # Publisher
        self.pub = self.create_publisher(Float32, 'proximity', 10)

        # Initialize state
        distance_min = self.get_parameter('distance_min').value
        distance_max = self.get_parameter('distance_max').value
        self.current_distance = random.uniform(distance_min, distance_max)
        self.time_since_change = 0.0
        hold_min = self.get_parameter('hold_duration_min').value
        hold_max = self.get_parameter('hold_duration_max').value
        self.hold_duration = random.uniform(hold_min, hold_max)

        # Setup timers
        publish_rate = self.get_parameter('publish_rate').value
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(publish_rate, self.publish_distance)
        self.create_timer(update_rate, self.update_distance)
        self.get_logger().info(f"Proximity sensor started - initial distance: {self.current_distance:.1f}mm")

    def publish_distance(self):
        """Publish the current distance value"""
        msg = Float32(data=self.current_distance)
        self.pub.publish(msg)

    def update_distance(self):
        """Update distance value occasionally"""
        update_rate = self.get_parameter('update_rate').value
        self.time_since_change += update_rate

        if self.time_since_change >= self.hold_duration:
            old_distance = self.current_distance
            distance_min = self.get_parameter('distance_min').value
            distance_max = self.get_parameter('distance_max').value
            self.current_distance = random.uniform(distance_min, distance_max)
            self.time_since_change = 0.0

            hold_min = self.get_parameter('hold_duration_min').value
            hold_max = self.get_parameter('hold_duration_max').value
            self.hold_duration = random.uniform(hold_min, hold_max)
            self.get_logger().info(f"Distance changed: {old_distance:.1f}mm -> {self.current_distance:.1f}mm")


def main():
    rclpy.init()
    node = ProximitySensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()