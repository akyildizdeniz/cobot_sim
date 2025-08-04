#!/usr/bin/env python3

"""
ROS2 Integration Tests for Cobot Sim Package

This module provides comprehensive integration testing for the proximity-based
speed control system. Tests actual ROS2 nodes running as separate processes
and verifies end-to-end system behavior.

Test Categories:
- Node Discovery & Launch: Verify nodes can be found and started
- Basic Communication: Test ROS2 pub/sub between nodes
- Speed Control Logic: Verify proximity -> speed state transitions
- Emergency Stop System: Test e-stop override behavior
- File-based Interfaces: Test file monitoring functionality

Run with: colcon test --packages-select cobot_sim
"""

import unittest
import sys
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Optional
from std_msgs.msg import Float64MultiArray

# ------------------------------------------------------------------------------
#  Setup Python path so we can import cobot_sim package and messages
# ------------------------------------------------------------------------------
def setup_python_path() -> Path:
    """Configure Python path to find cobot_sim modules."""
    workspace_root = Path.cwd()
    while workspace_root.name != 'ros2_ws' and workspace_root.parent != workspace_root:
        workspace_root = workspace_root.parent

    install_paths = [
        workspace_root / 'install' / 'cobot_sim' / 'lib' / 'python3.12' / 'site-packages',
        Path('/home/ubuntu/ros2_ws/install/cobot_sim/lib/python3.12/site-packages'),
    ]
    for path in install_paths:
        if path.exists() and str(path) not in sys.path:
            sys.path.insert(0, str(path))

    return workspace_root

WORKSPACE_ROOT = setup_python_path()

# ------------------------------------------------------------------------------
#  Try to import rclpy and ROS2 message types
# ------------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32, Bool
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# SpeedState message (real if available, otherwise a simple mock)
try:
    from cobot_sim.msg import SpeedState
except ImportError:
    class SpeedState:
        FULL = 0
        SLOW = 1
        STOP = 2
        UNKNOWN = 3

# ------------------------------------------------------------------------------
#  Integration Test Suite
# ------------------------------------------------------------------------------
class IntegrationTest(unittest.TestCase):
    """
    Comprehensive integration tests for the cobot speed control system.

    These tests launch actual ROS2 nodes and verify system behavior including:
    - Node lifecycle management
    - Inter-node communication
    - Speed control state machine
    - Emergency stop functionality
    - File-based interfaces
    """

    @classmethod
    def setUpClass(cls):
        """One-time setup: init ROS2, discover node scripts, create estop file."""
        print("\nInitializing Integration Tests")
        cls.ros2_available = ROS2_AVAILABLE

        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                print("ROS2 initialized")
            except Exception as e:
                print(f"ROS2 initialization failed: {e}")
                cls.ros2_available = False

        cls.node_paths = cls._discover_nodes()
        print(f"Discovered nodes: {list(cls.node_paths.keys())}")

        # Create a temporary file for file-based emergency stop
        cls.estop_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        cls.estop_file.write('false\n')
        cls.estop_file.close()
        print(f"Emergency stop file: {cls.estop_file.name}")

    @classmethod
    def tearDownClass(cls):
        """One-time cleanup: remove temporary estop file."""
        if hasattr(cls, 'estop_file'):
            os.unlink(cls.estop_file.name)
        print("Integration test cleanup complete\n")

    @classmethod
    def _discover_nodes(cls) -> Dict[str, str]:
        """Discover all .py scripts under src/cobot_sim/nodes and installed package."""
        discovered: Dict[str, str] = {}

        # 1) Source tree
        src_nodes = WORKSPACE_ROOT / 'src' / 'cobot_sim' / 'nodes'
        if src_nodes.exists():
            for f in src_nodes.glob('*.py'):
                discovered[f.name] = str(f)

        # 2) Installed package
        install_nodes = (
            WORKSPACE_ROOT
            / 'install' / 'cobot_sim'
            / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}'
            / 'site-packages' / 'cobot_sim' / 'nodes'
        )
        if install_nodes.exists():
            for f in install_nodes.glob('*.py'):
                discovered[f.name] = str(f)

        return discovered

    def setUp(self):
        """Per-test setup: track subprocesses, create TestMonitorNode."""
        self.processes: List[subprocess.Popen] = []
        self.test_node: Optional[TestMonitorNode] = None
        self.executor: Optional[SingleThreadedExecutor] = None

        if self.ros2_available:
            try:
                self.test_node = TestMonitorNode()
                self.executor = SingleThreadedExecutor()
                self.executor.add_node(self.test_node)
            except Exception as e:
                print(f"Could not create TestMonitorNode: {e}")
                self.test_node = None

    def tearDown(self):
        """Per-test cleanup: terminate processes and reset estop file."""
        for p in self.processes:
            if p.poll() is None:
                try:
                    p.send_signal(signal.SIGINT)
                    p.wait(timeout=3)
                except Exception:
                    p.kill()

        if self.test_node:
            self.test_node.destroy_node()

        # Reset the contents of the emergency stop file
        with open(self.estop_file.name, 'w') as f:
            f.write('false\n')

    def launch_node(self, node_filename: str) -> subprocess.Popen:
        """
        Launch a ROS2 node script, overriding estop_file_path for the monitor.

        Args:
            node_filename: name of the .py file under cobot_sim/nodes
        Returns:
            The subprocess.Popen handle.
        """
        if node_filename not in self.node_paths:
            self.skipTest(f"Node {node_filename} not available")

        node_path = self.node_paths[node_filename]
        cmd = ['python3', node_path]

        # If launching the Emergency Stop Monitor, pass our temp file path
        if node_filename == 'emergency_stop_monitor_node.py':
            cmd += [
                '--ros-args',
                '-p', f'estop_file_path:={self.estop_file.name}'
            ]

        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '0'
        env['PYTHONPATH'] = (
            f"{WORKSPACE_ROOT}/install/cobot_sim/lib/python3.12/site-packages:"
            + env.get('PYTHONPATH', '')
        )

        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.processes.append(proc)
        return proc

    def wait_for_nodes_ready(self, timeout: float = 5.0) -> None:
        """
        Spin briefly so TestMonitorNode can see publishers.

        Args:
            timeout: total seconds to spin the executor
        """
        if not self.test_node:
            return
        end_time = time.time() + timeout
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)

    # --------------------------------------------------------------------------
    # Node Discovery & Launch Tests
    # --------------------------------------------------------------------------

    def test_node_discovery(self):
        """Verify that the critical node scripts are present."""
        print("Test: Node Discovery")
        required = ['speed_controller_node.py', 'emergency_stop_monitor_node.py']
        for name in required:
            with self.subTest(node=name):
                self.assertIn(name, self.node_paths, f"{name} not found")
                path = Path(self.node_paths[name])
                self.assertTrue(path.is_file(), f"{name} is not a file")
        print("Node discovery passed.\n")

    def test_multiple_node_launch(self):
        """
        Launch speed_controller_node.py and emergency_stop_monitor_node.py
        simultaneously and verify they stay alive.
        """
        print("Test: Multiple Node Launch")
        to_launch = ['speed_controller_node.py', 'emergency_stop_monitor_node.py']
        available = [n for n in to_launch if n in self.node_paths]
        if len(available) < 2:
            self.skipTest("Both speed_controller_node.py and emergency_stop_monitor_node.py required")
        for node in available:
            self.launch_node(node)
        time.sleep(3.0)
        running = sum(1 for p in self.processes if p.poll() is None)
        self.assertEqual(running, len(available), "Not all nodes stayed running")
        print("Multiple node launch passed.\n")

    # --------------------------------------------------------------------------
    # Basic Communication Test
    # --------------------------------------------------------------------------

    def test_basic_communication(self):
        """
        Ensure proximity -> speed_state messages flow end-to-end
        via the TestMonitorNode.
        """
        print("Test: Basic Communication")
        if not ROS2_AVAILABLE:
            self.skipTest("rclpy not available")

        self.launch_node('speed_controller_node.py')
        self.launch_node('emergency_stop_monitor_node.py')
        time.sleep(3.0)
        self.wait_for_nodes_ready()

        # Send one proximity reading
        self.test_node.publish_proximity(600.0)
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.1)

        stats = self.test_node.get_stats()
        self.assertGreater(
            stats['speed_state_messages'], 0,
            "No speed_state messages received"
        )
        print("Basic communication test passed.\n")

    # --------------------------------------------------------------------------
    # Speed Control Logic Tests
    # --------------------------------------------------------------------------
    def test_speed_control_sequence(self):
        """
        Validate correct state transitions for a series of proximity values.

        The very first reading (1000 mm) will usually not publish a state
        because the controller starts in UNKNOWN and doesn't change.  We
        therefore accept no message for that first input.
        """
        print("Test: Speed Control Sequence")
        if not ROS2_AVAILABLE:
            self.skipTest("rclpy not available")

        # Launch the nodes
        self.launch_node('speed_controller_node.py')
        self.launch_node('emergency_stop_monitor_node.py')
        time.sleep(3.0)
        self.wait_for_nodes_ready()

        test_cases = [
            # (distance, allowed_states, description)
            (1000.0, [SpeedState.UNKNOWN, SpeedState.FULL], "Initial region"),
            (600.0,  [SpeedState.SLOW],                "SLOW region"),
            (300.0,  [SpeedState.STOP],                "STOP region"),
            (500.0,  [SpeedState.SLOW],                "Back to SLOW"),
            (900.0,  [SpeedState.FULL],                "Back to FULL"),
        ]

        for distance, expected_states, desc in test_cases:
            self.test_node.clear_stats()
            self.test_node.publish_proximity(distance)

            received = None
            # spin up to 3 seconds
            for _ in range(30):
                self.executor.spin_once(timeout_sec=0.1)
                if self.test_node.last_speed_state is not None:
                    received = self.test_node.last_speed_state
                    break

            if desc == "Initial region":
                # The first input may not produce any state message, by the design of our controller.
                if received is None:
                    print("  Initial region: no state change published (as expected)")
                    continue

            self.assertIn(
                received, expected_states,
                f"For proximity {distance}mm ({desc}) expected {expected_states}, got {received}"
            )
            print(f"  {desc} ({distance}mm) -> {received} OK")

        print("Speed control sequence test passed.\n")

    # --------------------------------------------------------------------------
    # Emergency Stop Tests
    # --------------------------------------------------------------------------

    def test_emergency_stop_gui(self):
        """Test GUI-based emergency-stop activation and clearing."""
        print("Test: GUI Emergency Stop")
        if not ROS2_AVAILABLE:
            self.skipTest("rclpy not available")

        self.launch_node('emergency_stop_monitor_node.py')
        time.sleep(2.0)
        self.wait_for_nodes_ready()

        # Trigger via 'estop_gui' topic
        self.test_node.clear_stats()
        self.test_node.publish_estop_gui(True)
        activated = False
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.1)
            if self.test_node.last_estop is True:
                activated = True
                break
        self.assertTrue(activated, "Emergency stop not activated via GUI")

        # Clear via 'estop_gui'
        self.test_node.clear_stats()
        self.test_node.publish_estop_gui(False)
        cleared = False
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.1)
            if self.test_node.last_estop is False:
                cleared = True
                break
        self.assertTrue(cleared, "Emergency stop not cleared via GUI")
        print("GUI emergency-stop test passed.\n")

    def test_file_based_emergency_stop(self):
        """Test file-based emergency-stop activation and clearing."""
        print("Test: File-based Emergency Stop")

        self.launch_node('emergency_stop_monitor_node.py')
        time.sleep(2.0)
        self.wait_for_nodes_ready()

        # Activate via file
        self.test_node.clear_stats()
        with open(self.estop_file.name, 'w') as f:
            f.write('true\n')
        activated = False
        for _ in range(30):
            self.executor.spin_once(timeout_sec=0.1)
            if self.test_node.last_estop is True:
                activated = True
                break
        self.assertTrue(activated, "File-based emergency stop activation failed")

        # Clear via file
        self.test_node.clear_stats()
        with open(self.estop_file.name, 'w') as f:
            f.write('false\n')
        cleared = False
        for _ in range(30):
            self.executor.spin_once(timeout_sec=0.1)
            if self.test_node.last_estop is False:
                cleared = True
                break
        self.assertTrue(cleared, "File-based emergency stop clearing failed")
        print("File-based emergency-stop test passed.\n")

    def test_emergency_stop_propagation_to_robot(self):
        """
        Integration: ensure an emergency stop cascades through the system:
        Monitor -> SpeedController -> RobotSpeedControlNode,
        and that the robot publishes zero velocities.
        """
        print("Test: Emergency Stop Propagation to Robot")
        if not ROS2_AVAILABLE:
            self.skipTest("rclpy not available")

        # 1) Launch monitor, speed controller, and robot controller
        self.launch_node('emergency_stop_monitor_node.py')
        self.launch_node('speed_controller_node.py')
        self.launch_node('robot_speed_control_node.py')
        time.sleep(3.0)
        self.wait_for_nodes_ready()

        # 2) Publish a mid-range proximity to force SLOW
        self.test_node.clear_stats()
        self.test_node.publish_proximity(600.0)   # inside slow_in, outside stop_in
        saw_slow = False
        for _ in range(30):
            self.executor.spin_once(timeout_sec=0.1)
            if self.test_node.last_speed_state == SpeedState.SLOW:
                saw_slow = True
                break
        self.assertTrue(saw_slow, "SpeedController did not enter SLOW state")

        # 3) Now fire GUI e-stop
        self.test_node.clear_stats()
        self.test_node.publish_estop_gui(True)

        # 4) Await STOP publication and zero-velocity command
        saw_stop = False
        saw_zero_vel = False
        for _ in range(30):
            self.executor.spin_once(timeout_sec=0.1)
            if not saw_stop and self.test_node.last_speed_state == SpeedState.STOP:
                saw_stop = True
            if self.test_node.velocity_count > 0:
                saw_zero_vel = True
                break

        self.assertTrue(saw_stop,    "SpeedController did not publish STOP")
        self.assertTrue(saw_zero_vel, "RobotSpeedControlNode did not send zero velocities")

        # 5) Check that the last velocity command is actually all zeros
        vel = self.test_node.last_velocity_command
        self.assertIsNotNone(vel, "No velocity command captured")
        self.assertEqual(len(vel), 6, "Expected 6 joint velocities")
        self.assertTrue(all(v == 0.0 for v in vel),
                        f"Expected all zeros, got {vel}")

        print("Emergency-stop propagation test passed.\n")


    # --------------------------------------------------------------------------
    # System Integration Smoke Test
    # --------------------------------------------------------------------------

    def test_full_system_integration(self):
        """Smoke test: launch all critical nodes and ensure they stay alive."""
        print("Test: Full System Integration")
        critical = ['speed_controller_node.py', 'emergency_stop_monitor_node.py']
        count = 0
        for n in critical:
            if n in self.node_paths:
                self.launch_node(n)
                count += 1

        time.sleep(4.0)
        running = sum(1 for p in self.processes if p.poll() is None)
        self.assertGreaterEqual(running, count, "Not all critical nodes stayed running")
        print("Full system integration smoke test passed.\n")



# ------------------------------------------------------------------------------
#  Helper TestMonitorNode
# ------------------------------------------------------------------------------
class TestMonitorNode(Node):
    """
    Test monitor node for integration tests.

    Publishes test inputs (proximity, estop_gui) and subscribes to system outputs
    (proximity echoes, estop, speed_state) so tests can observe behavior.
    """
    def __init__(self):
        super().__init__('test_monitor_node')

        # Counters and last values
        self.proximity_count = 0
        self.speed_state_count = 0
        self.estop_count = 0
        self.last_speed_state = None
        self.last_estop = None

        # Publishers for test commands
        self.proximity_pub = self.create_publisher(Float32, 'proximity', 10)
        self.estop_gui_pub = self.create_publisher(Bool,   'estop_gui', 10)

        # Subscribers for system outputs
        self.create_subscription(Float32, 'proximity',    self._on_proximity,    10)
        self.create_subscription(Bool,    'estop',        self._on_estop,         10)

        # subscribe to robot zero-velocity commands
        self.velocity_count = 0
        self.last_velocity_command: Optional[List[float]] = None
        self.create_subscription(
            Float64MultiArray,
            '/joint_group_velocity_controller/commands',
            self._on_velocity_command,
            10
        )

        # Subscribe to SpeedState if available, else fallback to Int32
        self.create_subscription(SpeedState, 'speed_state', self._on_speed_state, 10)

        self.get_logger().info("TestMonitorNode initialized")

    def _on_proximity(self, msg: Float32):
        self.proximity_count += 1

    def _on_speed_state(self, msg):
        # SpeedState has attribute current_state; don't assume msg.data exists
        if hasattr(msg, 'current_state'):
            self.last_speed_state = msg.current_state
        else:
            self.last_speed_state = msg.data
        self.speed_state_count += 1

    def _on_speed_state_fallback(self, msg):
        # Fallback for Int32
        self.last_speed_state = msg.data
        self.speed_state_count += 1

    def _on_estop(self, msg: Bool):
        self.last_estop = msg.data
        self.estop_count += 1

    def publish_proximity(self, distance: float):
        msg = Float32(data=distance)
        self.proximity_pub.publish(msg)

    def publish_estop_gui(self, active: bool):
        msg = Bool(data=active)
        self.estop_gui_pub.publish(msg)

    def _on_velocity_command(self, msg: Float64MultiArray):
        """Handle zero-velocity commands from the robot."""
        self.last_velocity_command = list(msg.data)
        self.velocity_count += 1

    def clear_stats(self):
        """Reset message counters and last values."""
        self.proximity_count = 0
        self.speed_state_count = 0
        self.estop_count = 0
        self.velocity_count = 0
        self.last_speed_state = None
        self.last_estop = None
        self.last_velocity_command = None

    def get_stats(self) -> Dict[str, any]:
        """Get current message statistics."""
        return {
            'proximity_messages': self.proximity_count,
            'speed_state_messages': self.speed_state_count,
            'estop_messages': self.estop_count,
            'velocity_messages': self.velocity_count,
            'last_speed_state': self.last_speed_state,
            'last_estop': self.last_estop,
            'last_velocity_command': self.last_velocity_command,
        }


if __name__ == '__main__':
    unittest.main(verbosity=2)

