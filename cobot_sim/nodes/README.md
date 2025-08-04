# Node Reference for `cobot_sim`

A summary of each node’s purpose, topics, and parameters.


---

## 1. Proximity Sensor Node  
**File**: `nodes/proximity_sensor_node.py`

- **Publishes**  
  - `proximity` (`std_msgs/Float32`): current distance (mm)
- **Parameters**  
  - `distance_min`, `distance_max`: sensor range  
  - `publish_rate`, `update_rate`: timers  
  - `hold_duration_min`, `hold_duration_max`: how long to hold a reading
- **Behavior**  
  Randomly varies the distance, holds it for a random duration, and publishes at `publish_rate`.

---

## 2. Speed Controller Node  
**File**: `nodes/speed_controller_node.py`

- **Subscribes**  
  - `proximity` (`std_msgs/Float32`)  
  - `estop` (`std_msgs/Bool`)
- **Publishes**  
  - `speed_state` (`cobot_sim/SpeedState`) (custom message format, could also have been just an integer and decoded on the subscriber side, but thought this is cleaner)
- **Parameters**  
  - `slow_in` / `slow_out`, `stop_in` / `stop_out` (mm)  
  - `control_period` (seconds)
- **Behavior**  
  Implements a four-state state machine with two hysteresis buffers:
  - `UNKNOWN/FULL` -> `SLOW` if `prox ≤ slow_in`
  - `SLOW` -> `STOP` if `prox <= stop_in`
  - `SLOW` -> `FULL` if `prox > slow_out`
  - `STOP` -> `SLOW` if `prox > stop_out` (then `SLOW`->`FULL` on next tick)
  - Emergency‐stop override: any state -> `STOP`.

---

## 3. Emergency Stop Monitor Node  
**File**: `nodes/emergency_stop_monitor_node.py`

- **Subscribes**  
  - `estop_gui` (`std_msgs/Bool`)  
  - `estop_hardware` (`std_msgs/Bool`)
- **Publishes**  
  - `estop` (`std_msgs/Bool`)
- **Parameters**  
  - `file_check_period` (seconds)  
  - `estop_file_path` (string)
- **Behavior**  
  Aggregates GUI, file-based, and hardware (only here to scale if needed, not used) e-stop inputs. If any is active, publishes `estop=true`.

---

## 4. Robot Motion Control Node  
**File**: `nodes/robot_motion_control_node.py`

- **Subscribes**  
  - `speed_state` (`cobot_sim/SpeedState`)  
  - `joint_states` (`sensor_msgs/JointState`)
- **Publishes**  
  - `/joint_group_velocity_controller/commands` (`std_msgs/Float64MultiArray`)
- **Action Client**  
  - `/scaled_joint_trajectory_controller/follow_joint_trajectory` (`FollowJointTrajectory`)
- **Parameters**  
  Defined in `config/robot_params.yaml`:  
  speed scales, joint positions A/B, thresholds, timing bounds.
- **Behavior**  
  - Alternates between two joint poses (A <-> B).  
  - Computes trajectory duration based on distance & speed scale.  
  - Cancels & replans on speed‐state changes.  
  - On e-stop, cancels goals and publishes zero velocities.

---

## 5. State Logger Node  
**File**: `nodes/state_logger_node.py`

- **Subscribes**  
  - `estop` (`std_msgs/Bool`)  
  - `speed_state` (`cobot_sim/SpeedState`)
- **Behavior**  
  - Opens (or creates) speed_log.csv and writes header: timestamp, event_type, speed_state, estop
  - On each estop message, if the boolean value changes, logs a row with event_type = estop_triggered or estop_cleared.
  - On each speed_state message, if the state value changes, logs a row with event_type = speed_change.
  - Flushes to disk immediately after each write for not losing data.

  See example below, you can run less speed_log.csv from root directory (ros2_ws) of docker.
    timestamp,event_type,speed_state,estop
    1754329580.0053217,speed_change,1,
    1754329580.0550976,speed_change,2,
    1754329582.1050541,speed_change,1,
    1754329582.1550467,speed_change,0,
    1754329586.008567,speed_change,1,
    1754329586.0550332,speed_change,2,
    1754329587.404974,speed_change,1,
    1754329589.9592106,speed_change,2,
    1754329592.3057995,speed_change,1,
    1754329592.3550563,speed_change,0,
    1754329593.9050465,speed_change,1,
    1754329594.078423,estop_triggered,1,True
    1754329594.1068203,speed_change,2,True
    1754329598.4783335,estop_cleared,2,False
    1754329598.605051,speed_change,1,False
    1754329598.655245,speed_change,0,False
    1754329601.6050813,speed_change,1,False
    1754329605.7051,speed_change,0,False
    1754329608.7051,speed_change,1,False
    1754329609.9049633,speed_change,0,False
    1754329612.1050048,speed_change,1,False
    1754329617.1050568,speed_change,2,False

---

## 5. Visualizer GUI Node  
**File**: `nodes/visualizer_node.py`

- **Subscribes**  
  - `estop` (`std_msgs/Bool`) 
  - `joint_states` (`sensor_msgs/JointState`)
  - `proximity` (`std_msgs/Float32`)  
  - `speed_state` (`cobot_sim/SpeedState`)
- **Behavior**  
  - Proximity: Shows numeric and progress-bar (0-1200 mm) with colored “STOP/SLOW/FULL” zones.
  - Speed State: Large label colored by state (STOP = red, SLOW = orange, FULL = green, UNKNOWN = gray).
  - Emergency Stop:
    - Indicator label and background color reflect status.
    - Two buttons to trigger or clear e-stop, publishing on estop_gui.
  - Joint Velocities: Displays six joint velocity values, color-coded by magnitude.
  - Update Loop: GUI refreshes every 50 ms; ROS spinning runs in a background thread.
  - Shutdown: Gracefully stops ROS node on exit.
