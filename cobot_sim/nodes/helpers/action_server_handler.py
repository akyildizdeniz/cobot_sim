"""
This class includes the helper methods for robot motion control node.
It is split from that node to show clear separation of concerns, and easier maintainability.
"""

from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float64MultiArray

class ActionServerHandler:
    def __init__(self, node: Node):
        # grab the node so we log & access params/state
        self.node = node  
        # instantiate the client
        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.current_goal_handle = None
        self.is_moving = False

    def wait_for_server(self, timeout_sec: float = 10.0):
        if self.client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().info("Action server connected!")
        else:
            self.node.get_logger().warn("Action server not available. using velocity control")

    def send_trajectory(self, target_positions, speed_scale: float):
        """
        Send trajectory goal with intelligent duration scaling. Uses the same interface
        ur5 uses to send trajectories to the robot. we need to send a goal and duration 
        it should take to reach the goal. the rest will be handled by ur's control mechanism.
        
        Features:
        - Distance-based duration calculation
        - Speed scaling
        - Comprehensive error handling
        - Safety checks
        
        Args:
            target_positions: Joint positions to move to
            speed_scale: Speed scaling factor (higher = faster)
        """
        n = self.node
        if not self.client.server_is_ready():
            n.get_logger().debug("Action server not ready")
            return
        # Safety check. should not happen due to motion_control_loop checks either way.
        if speed_scale <= 0.0:
            n.get_logger().warn("Not sending trajectory, invalid speed scale")
            return
        # Prevent overlapping trajectories
        if self.is_moving:
            n.get_logger().debug("Already moving, skipping trajectory")
            return

        try:
            # Calculate duration based on distance
            duration = self._calculate_duration(target_positions, speed_scale)
            # Create trajectory goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            goal.trajectory.points = [point]
            # Send goal and track state
            n.get_logger().info(
                f"Sending trajectory (duration: {duration:.2f}s, speed: {speed_scale:.1f}x)"
            )
            # Maybe we need to actually read from the joints to see if it is moving, but for now we are assuming
            # this works. since if it does not work, we set it to false in exception handling, and we also handle in the callback.
            self.is_moving = True
            send_goal_future = self.client.send_goal_async(goal)
            send_goal_future.add_done_callback(self._on_goal_response)

        except Exception as e:
            n.get_logger().error(f"Error sending trajectory: {e}")
            self.is_moving = False

    def cancel_current_goal(self):
        """
        Cancel current trajectory execution safely
        
        Handles:
        - Graceful trajectory cancellation
        - State cleanup
        - Error handling
        """
        try:
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            # Reset movement state immediately
            self.is_moving = False
            self.current_goal_handle = None
        except Exception as e:
            self.node.get_logger().error(f"Error cancelling trajectory: {e}")
            # Force reset state even if cancel failed
            self.is_moving = False
            self.current_goal_handle = None

    def _calculate_duration(self, target_positions, speed_scale):
        """
        Calculate intelligent trajectory duration based on distance and speed scale.
        Args:
            target_positions: Target joint positions
            speed_scale: Speed scaling factor
        Returns:
            float: Duration in seconds
        """
        try:
            cur = self.node.current_joint_positions
            if cur and len(cur) >= self.node.num_joints:
                # Calculate maximum joint movement (use worst case for timing)
                max_joint_movement = 0.0
                for i in range(self.node.num_joints):
                    current_pos = cur[i]
                    target_pos = target_positions[i]
                    movement = abs(target_pos - current_pos)
                    if movement > max_joint_movement:
                        max_joint_movement = movement

                # Base duration calculation
                base_joint_speed = self.node.get_parameter('base_joint_speed').value
                base_duration = max_joint_movement / base_joint_speed

                # Apply speed scaling
                scaled_duration = base_duration / max(speed_scale, 0.1)

                # Enforce reasonable bounds
                min_duration = self.node.get_parameter('min_duration').value
                max_duration = self.node.get_parameter('max_duration').value
                final_duration = max(min_duration, min(scaled_duration, max_duration))

                self.node.get_logger().debug(
                    f"Duration calc: movement={max_joint_movement:.3f}rad, "
                    f"base={base_duration:.2f}s, scaled={scaled_duration:.2f}s, "
                    f"final={final_duration:.2f}s"
                )
                return final_duration

            else:
                # Fallback to fixed duration if no position data
                self.node.get_logger().debug("No position data, using fixed duration")
                fallback_duration = self.node.get_parameter('fallback_duration').value
                return fallback_duration / max(speed_scale, 0.1)

        except Exception as e:
            self.node.get_logger().error(f"Error calculating duration: {e}")
            # Safe fallback
            safe_fallback = self.node.get_parameter('safe_fallback').value
            return safe_fallback / max(speed_scale, 0.1)

    def _on_goal_response(self, future):
        """
        Handle trajectory goal response with comprehensive error handling
        """
        n = self.node
        try:
            handle = future.result()
            if handle.accepted:
                self.current_goal_handle = handle
                # Completion callback
                res_fut = handle.get_result_async()
                res_fut.add_done_callback(self._on_goal_complete)
            else:
                n.get_logger().warn("Trajectory goal rejected")
                self.is_moving = False
                self.current_goal_handle = None
        except Exception as e:
            n.get_logger().error(f"Error in goal response: {e}")
            self.is_moving = False
            self.current_goal_handle = None

    def _on_goal_complete(self, future):
        """
        Handle trajectory completion with proper state cleanup
        """
        # Always reset movement state
        n = self.node
        self.is_moving = False
        self.current_goal_handle = None
        try:
            result = future.result()
            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                n.get_logger().debug("Trajectory completed successfully")
            elif status == GoalStatus.STATUS_CANCELED:
                n.get_logger().debug("Trajectory was cancelled")
            elif status == GoalStatus.STATUS_ABORTED:
                n.get_logger().warn("Trajectory was aborted by controller")
            else:
                n.get_logger().warn(f"Trajectory completed with status: {status}")
        except Exception as e:
            n.get_logger().error(f"Error in goal completion: {e}")
