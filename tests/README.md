# Testing Guide for `cobot_sim`

How to run and interpret the automated test suites.

## Overview

- **Integration Tests** (`test_integration.py`):  
  End-to-end verification by launching real ROS2 nodes.
- **Unit Tests** (`test_unit.py`):  
  Fast, isolated tests of individual node logic (mocking ROS2).

## Prerequisites

1. Inside `ros2_ur5_interface` container.  
2. Package built at least once:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select cobot_sim
   source install/setup.bash
   ```
3. Run tests:
   For unit tests:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select cobot_sim && source install/setup.bash && python3 -m pytest src/cobot_sim/tests/test_unit.py -v
   ```

   For integration tests:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select cobot_sim && source install/setup.bash && python3 -m pytest src/cobot_sim/tests/test_integration.py -v
   ```

## Key Integration Checks
- Node Discovery & Launch
- Basic Communication
- Speed Control Sequence
- GUI & File Emergency-Stop
- E-Stop Propagation to Robot (tests speed and motion control nodes too.)

## Key Unit Checks
- SpeedController Logic (thresholds, hysteresis, e-stop override)
- EmergencyStopMonitor Logic (aggregation, file parsing)
- Fast & Deterministic—no external processes.
