"""
This class includes the helper methods for robot motion control node.
It is split from that node to show clear separation of concerns, and easier maintainability.
"""

from typing import List
from rclpy.node import Node

class TargetPlanner:
    """
    Encapsulates A/B alternation logic for picking a target for demo purposes.

    Attributes:
        position_a (List[float]): Joint positions for Position A
        position_b (List[float]): Joint positions for Position B
        threshold (float): Distance threshold to consider close enough
        _target_is_b (bool): Internal toggle for which target is current
    """
    def __init__(self,
                node: Node,
                position_a: List[float],
                position_b: List[float],
                proximity_threshold: float):
        self.node = node
        self.position_a = position_a
        self.position_b = position_b
        self.threshold = proximity_threshold
        self._target_is_b = True

    def next_target(self, current_joint_positions: List[float]) -> List[float]:
        """
        Plan next target - stick with current target until reached, then switch
        This ensures we complete movements even if interrupted by speed changes.
        Only switches target when we've actually reached the current target.

        NOTE: this function is actually a throw away logic, which shoud be replaced
        with the actual controller of the robot. I only wrote it to showcase
        that the core logic asked for in the requirements is working.

        Returns:
            List[float]: Target joint positions
        """
        # Fallback if no position data present
        if current_joint_positions is None:
            return self._get_simple_alternating_target()

        try:
            # Read base joint (index 0) position
            current_base_pos = current_joint_positions[0]
            position_a_base = self.position_a[0]
            position_b_base = self.position_b[0]

            # Define "close enough" threshold
            proximity_threshold = self.threshold

            # Check if we are within range of either target
            close_to_a = abs(current_base_pos - position_a_base) < proximity_threshold
            close_to_b = abs(current_base_pos - position_b_base) < proximity_threshold

            # Determine which target we are currently pursuing
            if self._target_is_b:
                current_target_name = "Position B"
                current_target_pos = self.position_b.copy()
                alternative_target_name = "Position A"
                alternative_target_pos = self.position_a.copy()
                target_reached = close_to_b
            else:
                current_target_name = "Position A"
                current_target_pos = self.position_a.copy()
                alternative_target_name = "Position B"
                alternative_target_pos = self.position_b.copy()
                target_reached = close_to_a

            # If we've reached the current target, switch to the other one
            if target_reached:
                self._target_is_b = not self._target_is_b
                self.node.get_logger().info(
                    f"Reached {current_target_name}! Switching to {alternative_target_name}"
                )
                return alternative_target_pos
            else:
                # Otherwise, continue moving toward the current target
                distance_remaining = abs(current_base_pos - current_target_pos[0])
                self.node.get_logger().info(
                    f"Continuing to {current_target_name} "
                    f"(current: {current_base_pos:.3f}, "
                    f"target: {current_target_pos[0]:.3f}, "
                    f"remaining: {distance_remaining:.3f})"
                )
                return current_target_pos

        except Exception as e:
            self.node.get_logger().error(f"Error in target planning: {e}")
            return self._get_simple_alternating_target()

    def _get_simple_alternating_target(self) -> List[float]:
        """
        Fallback target planning. simple A/B alternation.

        Only switches the toggle flag and returns the corresponding preset
        target. Used when no joint data is available or in error conditions.

        Returns:
            List[float]: The joint positions for the chosen fallback target.
        """
        if self._target_is_b:
            self.node.get_logger().info("Moving to Position B (fallback planning)")
            result = self.position_b.copy()
        else:
            self.node.get_logger().info("Moving to Position A (fallback planning)")
            result = self.position_a.copy()

        # Toggle for next call
        self._target_is_b = not self._target_is_b
        return result
