# RoboPostman

An autonomous delivery robot simulation built with ROS 2 Jazzy and Gazebo Harmonic. RoboPostman navigates a simulated neighborhood, picks up parcels from a depot, and delivers them to houses while avoiding obstacles.

---

## Features

- **Autonomous Navigation** using Nav2 and SLAM Toolbox for real-time map building
- **Manual Control** via keyboard teleop with seamless mode switching
- **Parcel Pickup & Delivery** state machine with 4 delivery houses
- **Dynamic Obstacle Spawning** — coins, fuel canisters, potholes, and dogs spawn near the robot
- **Scoring System** — tracks deliveries, coins, fuels collected, potholes and dogs hit
- **Coverage Tracking** — calculates map exploration percentage

---

## System Architecture

```
Gazebo Sim (neighborhood.sdf)
       ↓
ros_gz_bridge
  ├── /scan        → SLAM Toolbox → /map
  ├── /odom        → Mission Manager
  ├── /cmd_vel     ← Mission Manager / Teleop
  └── /camera/image_raw → Camera Detector

Mission Manager (/mission_manager)
  ├── Subscribes: /odom, /mode_switch, /detection_event
  ├── Publishes:  /cmd_vel, /mission_hud, /mission_status
  └── Action:     navigate_to_pose (Nav2)

Obstacle Spawner (/obstacle_spawner)
  ├── Spawns obstacles near robot position
  ├── Detects proximity contact
  └── Publishes: /score_event, /detection_event

Scoring Trail (/scoring_trail)
  ├── Subscribes: /score_event, /deliveries_completed, /odom, /map
  └── Publishes:  /total_score, /score_detail, /coverage_percent
```

---

## Package Structure

```
src/robopostman/
├── config/
│   ├── nav2_params.yaml          # Nav2 navigation parameters
│   ├── slam_toolbox_params.yaml  # SLAM configuration
│   └── waypoints.yaml            # Delivery house coordinates
├── launch/
│   ├── robopostman.launch.py     # Full system launch
│   └── slam_only.launch.py       # SLAM-only launch for mapping
├── robopostman/
│   ├── mission_manager.py        # Core delivery state machine
│   ├── obstacle_spawner.py       # Dynamic obstacle management
│   ├── scoring_trail.py          # Score tracking and trail viz
│   ├── camera_detector.py        # OpenCV HSV obstacle detection
│   ├── keyboard_teleop.py        # Manual keyboard control
│   └── mode_switch_gui.py        # Terminal mode switcher
├── urdf/
│   └── robopostman.urdf          # Robot model (diff-drive + lidar + camera)
├── worlds/
│   └── neighborhood.sdf          # Gazebo world with houses, roads, trees
└── rviz/
    └── robopostman.rviz          # RViz visualization config
```

---

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2 (`nav2_bringup`)
- SLAM Toolbox (`slam_toolbox`)
- `ros_gz_bridge`, `ros_gz_sim`
- `teleop_twist_keyboard`

---

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select robopostman
source install/setup.bash
```

---

## Running the Project

### Full Launch (Gazebo + SLAM + Nav2 + All Nodes)

**Terminal 1 — Gazebo + SLAM + Nav2 + Nodes:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robopostman robopostman.launch.py
```

**Terminal 2 — Nav2:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$HOME/ros2_ws/src/robopostman/config/nav2_params.yaml
```

**Terminal 3 — Set Initial Pose (required for Nav2):**
Use RViz **2D Pose Estimate** tool to set robot's starting position on the map.

### Manual Keyboard Control

```bash
# Switch to manual mode
ros2 topic pub /mode_switch std_msgs/msg/String "data: 'manual'" --once

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel_manual

# Switch back to autonomous
ros2 topic pub /mode_switch std_msgs/msg/String "data: 'auto'" --once
```

### Monitor Mission & Score

```bash
# Mission state
ros2 topic echo /mission_hud

# Score breakdown
ros2 topic echo /score_detail

# Total score
ros2 topic echo /total_score

---

## Robot Description

The robot (`robopostman.urdf`) is a differential drive robot with:
- **Base**: 0.5×0.3×0.15m box (blue)
- **Drive**: Two continuous wheels (radius 0.08m, separation 0.34m)
- **Caster**: Rear passive wheel
- **LiDAR**: 360° CPU lidar, 10Hz, 10m range
- **Max Speed**: 0.5 m/s linear, 1.0 rad/s angular

---

## Scoring System

| Event | Points |
|---|---|
| Parcel delivered | +100 |
| Coin collected | +20 |
| Fuel collected | +30 |
| Dog contact | -50 |
| Pothole contact | -10 |
| Map coverage | +2 per % |

---

## Waypoints (Delivery Houses)

| House | X | Y |
|---|---|---|
| Red House | 10.0 | 5.5 |
| Blue House | -10.0 | 5.5 |
| Green House | 10.0 | -5.5 |
| Yellow House | -10.0 | -5.5 |

---

## Code Reuse & New Functionality

### Existing Libraries/Frameworks Used
- **Nav2** — path planning, obstacle avoidance, goal navigation
- **SLAM Toolbox** — real-time mapping and localization
- **Gazebo Harmonic** — physics simulation, sensor rendering
- **ros_gz_bridge** — ROS2 ↔ Gazebo topic bridging
- **teleop_twist_keyboard** — base keyboard control

### New Functionality Developed
- **Mission State Machine** (`mission_manager.py`) — full pickup/delivery cycle with proximity-based arrival detection and Nav2 action client integration
- **Dynamic Obstacle Spawner** (`obstacle_spawner.py`) — robot-relative procedural spawning, contact detection, Gazebo model lifecycle management
- **Scoring & Trail System** (`scoring_trail.py`) — multi-event scoring, map coverage calculation, path visualization
- **Neighborhood World** (`neighborhood.sdf`) — custom Gazebo world with roads, sidewalks, 4 colored houses, trees, humans, delivery markers

---

## Authors

- Aarti Dashore — Seattle University, ROS 2 Robotics Course
