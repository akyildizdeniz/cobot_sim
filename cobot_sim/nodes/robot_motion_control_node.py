#!/usr/bin/env python3

"""
Robot Control Node for Proximity-Based Safety System

This module translates speed control states into actual robot motion,
implementing safety measures and trajectory control for a UR5 cobot.

Key Features:
- Two-position movement pattern for demonstration
- Speed scaling via trajectory timing
- Immediate emergency stop with trajectory cancellation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cobot_sim.msg import SpeedState
from action_msgs.msg import GoalStatus
from cobot_sim.nodes.helpers.action_server_handler import ActionServerHandler
from cobot_sim.nodes.helpers.target_planner import TargetPlanner

import math
import threading


class RobotMotionControlNode(Node):
    def __init__(self):
        super().__init__('robot_motion_control')
        
        # Declare parameters here, but we don't need to have any hardcoded value. All the parameters can be declared in
        # config/robot_params.yaml file.
        self.declare_parameter('speed_scales.full', 3.0)
        self.declare_parameter('speed_scales.slow', 0.5)
        self.declare_parameter('speed_scales.stop', 0.0)
        self.declare_parameter('speed_scales.unknown', 0.0)
        self.declare_parameter('base_joint_speed', 0.1)
        self.declare_parameter('min_duration', 0.5)
        self.declare_parameter('max_duration', 10.0)
        self.declare_parameter('fallback_duration', 1.5)
        self.declare_parameter('safe_fallback', 2.0)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('position_a', [-1.60, -1.60, -2.20, -1.21, 1.60, -0.03])
        self.declare_parameter('position_b', [-1.40, -1.30, -1.95, -0.85, 1.20, -0.03])
        self.declare_parameter('proximity_threshold', 0.05)
        self.declare_parameter('speed_change_threshold', 0.1)
        # Safe position and motion parameters. Cache positions since they are called multiple times below.
        self.num_joints = self.get_parameter('num_joints').value
        self.position_a = self.get_parameter('position_a').value
        self.position_b = self.get_parameter('position_b').value
        # Which position we're moving toward
        self.current_target_is_b = True
        self.movement_started = False
        # State variables
        self.current_speed_state = SpeedState.UNKNOWN
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.joint_names = None
        self.robot_initialized = False
        self.robot_homed = False

        # Action client for trajectory control. Similar to publish_trajectory_node of ur5 interface
        self.action_handler = ActionServerHandler(self)

        # Target planner for alternating a/b goal selection
        self.target_planner = TargetPlanner(self, self.position_a, self.position_b,
                                    self.get_parameter('proximity_threshold').value)
        
        # Publisher for joint velocity commands, for stopping the robot.
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_velocity_controller/commands', 
            10
        )
        
        # Subscribers
        self.create_subscription(SpeedState, 'speed_state', self.speed_state_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for continuous motion control
        self.create_timer(1.0, self.motion_control_loop)

        self.get_logger().info("Robot Speed Control Node started")
        self.get_logger().info("Waiting for action server...")

        # Wait for action server in a separate thread to not block
        threading.Thread(target=self.action_handler.wait_for_server, daemon=True).start()



    def joint_state_callback(self, msg):
        """Store current joint state information"""
        if len(msg.position) >= self.num_joints:
            # Only use the main robot joints (exclude gripper joints)
            self.joint_names = [name for name in msg.name[:6] if 'gripper' not in name][:6]
            if len(self.joint_names) < self.num_joints:
                # Fallback to first 6 joints
                self.joint_names = msg.name[:self.num_joints]
            
            self.current_joint_positions = list(msg.position[:self.num_joints])
            self.current_joint_velocities = list(msg.velocity[:self.num_joints]) if msg.velocity else [0.0] * 6
            
            if not self.robot_initialized:
                self.robot_initialized = True
                self.robot_homed = True  # Skip homing
                self.movement_started = True  # Start moving immediately
                self.get_logger().info(f"Robot initialized! Skipping homing - starting movement")

    def speed_state_callback(self, msg):
        """Handle speed state changes"""
        old_state = self.current_speed_state
        self.current_speed_state = msg.current_state
        
        state_names = {
            SpeedState.FULL: "FULL_SPEED",
            SpeedState.SLOW: "SLOW", 
            SpeedState.STOP: "STOP",
            SpeedState.UNKNOWN: "UNKNOWN"
        }
        
        if old_state != self.current_speed_state:
            self.get_logger().info(
                f"Robot speed changed: {state_names.get(old_state, 'UNKNOWN')} -> {state_names.get(self.current_speed_state, 'UNKNOWN')}"
            )
            
            # If we need to stop immediately, cancel any ongoing motion
            if self.current_speed_state == SpeedState.STOP:
                self.emergency_stop()

    def emergency_stop(self):
        """Immediately stop all robot motion"""
        self.get_logger().warn("EMERGENCY STOP - Halting robot motion")
        
        # Send zero velocities
        zero_velocities = Float64MultiArray()
        zero_velocities.data = [0.0] * self.num_joints
        self.velocity_pub.publish(zero_velocities)        
        # important: Cancel any ongoing trajectory goals immediately
        self.action_handler.cancel_current_goal()
        # Force stop any movement flags
        self.action_handler.is_moving = False

    def motion_control_loop(self):
        """
        Main control loop for robot motion with intelligent replanning
        
        Handles:
        - Speed changes mid-trajectory
        - Emergency stops
        - Edge cases and safety checks
        """
        # Basic readiness checks
        if not self.robot_initialized or self.current_joint_positions is None or self.joint_names is None:
            return
        
        # Don't start motion control until robot is ready
        if not self.robot_homed or not self.movement_started:
            return
            
        # Get current speed scaling, default to zero
        speed_scale = {
            SpeedState.FULL: self.get_parameter('speed_scales.full').value,
            SpeedState.SLOW: self.get_parameter('speed_scales.slow').value,
            SpeedState.STOP: self.get_parameter('speed_scales.stop').value,
            SpeedState.UNKNOWN: self.get_parameter('speed_scales.unknown').value
        }.get(self.current_speed_state, 0.0)
        
        # Handle stop conditions (STOP, UNKNOWN, or E-STOP)
        if speed_scale == 0.0:
            self.send_zero_velocities()
            # If we were moving, cancel the trajectory
            if self.action_handler.is_moving:
                self.get_logger().info("Stopping motion due to speed state change")
                self.action_handler.cancel_current_goal()
            return

        # Handle speed changes during motion - cancel and replan
        if self.action_handler.is_moving and self._speed_changed(speed_scale):
            self.get_logger().info(f"Speed changed mid-trajectory (new scale: {speed_scale:.1f}) - replanning")
            self.action_handler.cancel_current_goal()
            # Continue to replanning below

        # Skip if still moving with same speed
        if self.action_handler.is_moving:
            return

        # Smart target planning based on current position
        target_positions = self.target_planner.next_target(self.current_joint_positions)

        # Send trajectory with speed scaling
        self.action_handler.send_trajectory(target_positions, speed_scale)

    def _speed_changed(self, current_speed_scale):
        """
        Check if speed scale changed significantly from last trajectory
        
        Args:
            current_speed_scale: Current speed scaling factor
            
        Returns:
            bool: True if speed changed enough to warrant replanning
        """
        # Initialize on first call
        if not hasattr(self, '_last_speed_scale'):
            self._last_speed_scale = current_speed_scale
            return False
        
        # Check for significant change
        threshold = self.get_parameter('speed_change_threshold').value
        changed = abs(self._last_speed_scale - current_speed_scale) > threshold
        
        # Update for next comparison
        self._last_speed_scale = current_speed_scale
        
        return changed

    def send_zero_velocities(self):
        """
        Send zero velocity commands to stop robot immediately
        
        This is used for emergency stops and when speed state is STOP
        """
        try:
            zero_velocities = Float64MultiArray()
            zero_velocities.data = [0.0] * self.num_joints
            self.velocity_pub.publish(zero_velocities)
            self.get_logger().debug("Sent zero velocities")
        except Exception as e:
            self.get_logger().error(f"Error sending zero velocities: {e}")

def main():
    rclpy.init()
    node = RobotMotionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()