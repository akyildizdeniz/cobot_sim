#!/usr/bin/env python3

"""
Unit Tests for Cobot Sim Package

This module provides isolated, fast‐running unit tests for individual components
of the proximity-based speed control system. We verify the core algorithms in
SpeedController and EmergencyStopMonitor without launching any ROS2 processes.

Test Categories:
- SpeedController Logic: State machine transitions and hysteresis
- EmergencyStopMonitor Logic: Input aggregation and file parsing

Run with:
    pytest test_unit.py
    or
    python3 -m unittest test_unit.py
"""

import os
import tempfile
import unittest
from unittest.mock import MagicMock

import rclpy

from std_msgs.msg import Bool

from cobot_sim.msg import SpeedState
from cobot_sim.nodes.speed_controller_node import SpeedController
from cobot_sim.nodes.emergency_stop_monitor_node import EmergencyStopMonitor

class TestSpeedControllerUnit(unittest.TestCase):
    """Unit tests for the SpeedController state machine logic."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy once for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy when all tests are done."""
        rclpy.shutdown()

    def setUp(self):
        """Create a SpeedController and replace its publisher with a mock."""
        self.node = SpeedController()
        # Replace the real publisher with a MagicMock to capture publishes
        self.node.pub = MagicMock()

    def test_initial_state_no_publish(self):
        """If no proximity and no estop, state remains UNKNOWN and no publish."""
        self.assertEqual(self.node.state, SpeedState.UNKNOWN)
        self.node.prox = None
        self.node.estop = False

        self.node.state_machine_tick()
        # state unchanged -> no publish
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.state, SpeedState.UNKNOWN)

    def test_unknown_to_slow(self):
        """UNKNOWN -> SLOW when proximity <= slow_in."""
        slow_in = self.node.get_parameter('slow_in').value
        self.node.prox = slow_in - 1.0
        self.node.state = SpeedState.UNKNOWN

        self.node.state_machine_tick()
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.state, SpeedState.SLOW)
        msg = self.node.pub.publish.call_args.args[0]
        self.assertEqual(msg.current_state, SpeedState.SLOW)

    def test_slow_to_full(self):
        """SLOW -> FULL when proximity > slow_out."""
        slow_out = self.node.get_parameter('slow_out').value
        self.node.prox = slow_out + 1.0
        self.node.state = SpeedState.SLOW

        self.node.state_machine_tick()
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.state, SpeedState.FULL)

    def test_slow_to_stop(self):
        """SLOW -> STOP when proximity <= stop_in."""
        stop_in = self.node.get_parameter('stop_in').value
        self.node.prox = stop_in - 1.0
        self.node.state = SpeedState.SLOW

        self.node.state_machine_tick()
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.state, SpeedState.STOP)

    def test_stop_to_slow(self):
        """STOP -> SLOW when proximity > stop_out and estop cleared."""
        stop_out = self.node.get_parameter('stop_out').value
        self.node.prox = stop_out + 1.0
        self.node.state = SpeedState.STOP
        self.node.estop = False

        self.node.state_machine_tick()
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.state, SpeedState.SLOW)

    def test_estop_override(self):
        """Any state -> STOP when estop is True."""
        # Start in FULL
        self.node.state = SpeedState.FULL
        self.node.prox = 1000.0
        self.node.estop = True

        self.node.state_machine_tick()
        self.node.pub.publish.assert_called_once()
        self.assertEqual(self.node.state, SpeedState.STOP)


class TestEmergencyStopMonitorUnit(unittest.TestCase):
    """Unit tests for EmergencyStopMonitor aggregation and file logic."""
    @classmethod
    def setUpClass(cls):
        """Initialize rclpy once for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy when all tests are done."""
        rclpy.shutdown()

    def setUp(self):
        """Instantiate the monitor and replace its publisher."""
        self.node = EmergencyStopMonitor()
        self.node.estop_pub = MagicMock()

        # Create a temporary file for file-based estop
        self.tmp = tempfile.NamedTemporaryFile(mode='w', delete=False)
        self.tmp.write('false\n')
        self.tmp.close()
        # Override the parameter for the file path
        self.node.get_parameter = lambda name: type('P', (), {'value': self.tmp.name})()

    def tearDown(self):
        """Remove the temp file after each test."""
        if os.path.exists(self.tmp.name):
            os.unlink(self.tmp.name)

    def test_aggregate_any_source(self):
        """ANY of gui, hardware, or file input -> current_estop = True."""
        # All clear initially
        self.node.gui_estop = False
        self.node.hardware_estop = False
        self.node.file_estop = False
        self.node.update_estop_state()
        self.node.estop_pub.publish.assert_not_called()
        self.assertFalse(self.node.current_estop)

        # GUI triggers
        self.node.gui_estop = True
        self.node.update_estop_state()
        self.node.estop_pub.publish.assert_called_once()
        self.assertTrue(self.node.current_estop)

    def test_file_based_parsing(self):
        """check_file_estop correctly reads truthy and falsy file contents."""
        # Test true variants
        for content in ['true', 'TRUE', '1', 'on', 'yes']:
            with open(self.tmp.name, 'w') as f:
                f.write(content + '\n')
            self.node.check_file_estop()
            self.assertTrue(self.node.file_estop, f"Failed parsing {content}")

        # Test false variants
        for content in ['false', 'FALSE', '0', 'off', 'no', '']:
            with open(self.tmp.name, 'w') as f:
                f.write(content + '\n')
            self.node.check_file_estop()
            self.assertFalse(self.node.file_estop, f"Failed parsing {content}")

    def test_missing_or_unreadable_file(self):
        """check_file_estop should not crash on missing or restricted files."""
        # Remove file
        os.unlink(self.tmp.name)
        self.node.check_file_estop()
        self.assertFalse(self.node.file_estop)

        # Create and remove read permissions
        with open(self.tmp.name, 'w') as f:
            f.write('true\n')
        os.chmod(self.tmp.name, 0)
        try:
            self.node.check_file_estop()
        except Exception:
            self.fail("check_file_estop() raised on permission error")
        finally:
            os.chmod(self.tmp.name, 0o644)


if __name__ == '__main__':
    unittest.main(verbosity=2)
