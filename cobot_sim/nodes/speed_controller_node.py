#!/usr/bin/env python3

"""
Speed Controller Node

Implements the state machine for proximity-based speed control with hysteresis.
Monitors proximity sensor data and emergency stop signals to determine appropriate
robot speed states.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from cobot_sim.msg import SpeedState


class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller') 

        # Declare parameters
        # We need this intermediate buffer states for transitioning properly
        # so if the proximity sensor is fluctuating at the border of one state so fast,
        # it doesnt cause speed changes. this way we have different conditions
        # for going in and out of state, meaning once the state changes it needs a more drastic change to 
        # go back to previous state immediately. 
        # We always have the required bounds in the document intact for slowing down, for safety.
        self.declare_parameter('slow_in', 800.0)
        self.declare_parameter('slow_out', 820.0)
        self.declare_parameter('stop_in', 400.0)
        self.declare_parameter('stop_out', 420.0)
        self.declare_parameter('control_period', 0.05)

        # State variables
        self.state = SpeedState.UNKNOWN
        self.prox = None
        self.estop = False

        # Setup subscribers and publisher
        self.create_subscription(Float32, 'proximity', self.proximity_callback, 10)
        self.create_subscription(Bool, 'estop', self.estop_callback, 10)
        self.pub = self.create_publisher(SpeedState, 'speed_state', 10)

        # Setup state machine timer
        control_period = self.get_parameter('control_period').value
        self.create_timer(control_period, self.state_machine_tick)

        self.get_logger().info("Speed Controller started")

        # Log threshold values
        slow_in = self.get_parameter('slow_in').value
        slow_out = self.get_parameter('slow_out').value
        stop_in = self.get_parameter('stop_in').value
        stop_out = self.get_parameter('stop_out').value

        self.get_logger().info(f"Thresholds - slow: {slow_in}/{slow_out}mm, stop: {stop_in}/{stop_out}mm")

    def proximity_callback(self, msg):
        """Handle proximity sensor readings"""
        self.prox = msg.data
        self.get_logger().debug(f"Received proximity: {self.prox:.1f}mm")

    def estop_callback(self, msg):
        """Handle emergency stop state changes"""
        old_estop = self.estop
        self.estop = msg.data
        if old_estop != self.estop:
            self.get_logger().info(f"Emergency stop {'ACTIVATED' if self.estop else 'CLEARED'}")

    def state_machine_tick(self):
        """
        State machine implementation with hysteresis.
        
        State transitions:
        - Emergency stop always takes precedence
        - Hysteresis prevents rapid oscillation at thresholds
        - Unknown state when no proximity data available
        """
        prev_state = self.state

        # Emergency stop overrides everything
        if self.estop:
            self.state = SpeedState.STOP
        elif self.prox is None:
            self.state = SpeedState.UNKNOWN
        else:
            # Get threshold parameters
            slow_in = self.get_parameter('slow_in').value
            slow_out = self.get_parameter('slow_out').value
            stop_in = self.get_parameter('stop_in').value
            stop_out = self.get_parameter('stop_out').value
            # Proximity-based Speed State Machine with Two-Level Hysteresis
            # States: UNKNOWN <-> FULL <-> SLOW <-> STOP

            # 1) UNKNOWN & FULL:
            # Treated identically for bootstrapping: on startup we begin in UNKNOWN,
            # which behaves like FULL in terms of "should I slow down?"
            # Only when proximity <= slow_in do we drop to SLOW.
            # We do NOT force UNKNOWN -> FULL on first reading, so no unsafe startup
            # publication.

            # 2) SLOW:
            # Two-way gate:
            #   – If proximity <= stop_in -> transition to STOP (object dangerously close).
            #   – If proximity > slow_out -> transition to FULL (object safely far away).
            # The gap between slow_in and slow_out (hysteresis band) prevents chatter
            # when the object hovers near the boundary.

            # 3) STOP:
            # The object is too close or emergency-stop is active. We stay in STOP
            # until it clears.
            # Once the global estop flag is false and proximity > stop_out, we
            # transition to SLOW, not directly to FULL, to enforce a controlled
            # ramp-up via the SLOW state.
            # On the next tick, since we are now in SLOW and if proximity > slow_out,
            # we then go SLOW -> FULL. That two-step ensures we never jump from STOP to
            # FULL in one go, which could be unsafe.

            # Safety & Stability Guarantees:
            # - No direct STOP -> FULL: Robot must retreat through SLOW, giving the system
            # a chance to re-evaluate and slow back if needed.
            # - Boot-safe: UNKNOWN behaves like FULL but does not generate an initial
            # FULL message—publications only happen on true state changes.
            # - Hysteresis bands (slow_in < slow_out, stop_in < stop_out):
            # Prevents oscillation when the sensor reading sits on a threshold.

            if self.state in (SpeedState.UNKNOWN, SpeedState.FULL):
                if self.prox <= slow_in:
                    self.state = SpeedState.SLOW
            elif self.state == SpeedState.SLOW:
                if self.prox <= stop_in:
                    self.state = SpeedState.STOP
                elif self.prox > slow_out:
                    self.state = SpeedState.FULL
            elif self.state == SpeedState.STOP and not self.estop:
                if self.prox > stop_out:
                    self.state = SpeedState.SLOW

        # Publish state changes
        if self.state != prev_state:
            msg = SpeedState(current_state=self.state)
            self.pub.publish(msg)
            state_names = {
                SpeedState.FULL: "FULL_SPEED",
                SpeedState.SLOW: "SLOW",
                SpeedState.STOP: "STOP", 
                SpeedState.UNKNOWN: "UNKNOWN"
            }
            self.get_logger().info(
                f"State transition: {state_names.get(prev_state, 'UNKNOWN')} -> {state_names.get(self.state, 'UNKNOWN')}"
            )

def main():
    rclpy.init()
    node = SpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()