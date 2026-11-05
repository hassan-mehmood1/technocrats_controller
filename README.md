# Technocrats F1TENTH Controller

## 1. Overview

This repository contains the ROS 2 controller package used by **Team Technocrats** for the F1TENTH autonomous racing platform.

The controller operates within the ROS 2 ecosystem and communicates with the F1TENTH simulator through standard ROS 2 topics.

## 2. Simulation Environment Setup

The simulation environment and initial ROS 2 setup were established using the following repository:

**CIE-KFUPM F1TENTH Autonomous Racing Competition**

[https://github.com/KayiiX/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition](https://github.com/KayiiX/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition)

Please refer to the original repository for simulation installation, environment configuration, and competition-specific instructions.

This repository contains the Technocrats controller package developed to operate within that environment.

## 3. Repository Structure

```text
technocrats_controller/
│
├── maps/
│   └── levine_centerline.csv
│
├── technocrats_controller/
│   ├── __init__.py
│   └── controller_node.py
│
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── technocrats_controller
│
├── .gitignore
└── README.md
```

## 4. Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- `colcon`
- F1TENTH simulation environment
- Required ROS 2 message packages:
  - `rclpy`
  - `sensor_msgs`
  - `nav_msgs`
  - `ackermann_msgs`
  - `geometry_msgs`

## 5. Installation

### Step 1: Clone the repository

Navigate to your ROS 2 workspace:

```bash
mkdir -p ~/controller_ws/src
cd ~/controller_ws/src
```

Clone the Technocrats controller repository:

```bash
git clone https://github.com/hassan-mehmood1/technocrats_controller.git
```

### Step 2: Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
```

### Step 3: Build the package

```bash
cd ~/controller_ws

colcon build --symlink-install --packages-select technocrats_controller
```

### Step 4: Source the workspace

```bash
source install/setup.bash
```

## 6. Running the Simulation

First, launch the F1TENTH simulation environment using the competition repository.

```bash
cd ~/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition
./scripts/launch_sim.sh
```

Keep the simulation running.

## 7. Running the Technocrats Controller

Open a new terminal and source ROS 2 and the controller workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/controller_ws/install/setup.bash
```

Run the controller:

```bash
ros2 run technocrats_controller controller --ros-args -p max_speed:=4.0
```

## 8. ROS 2 Topics

The controller interfaces with the simulator through the following topics:

| Topic               | Message Type                               | Purpose                                         |
| ------------------- | ------------------------------------------ | ----------------------------------------------- |
| `/scan`             | `sensor_msgs/msg/LaserScan`                | LiDAR measurements                              |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry`                    | Vehicle odometry and pose                       |
| `/drive`            | `ackermann_msgs/msg/AckermannDriveStamped` | Steering and speed commands                     |
| `/map`              | `nav_msgs/msg/OccupancyGrid`               | Map information, if published by the simulation |

## 9. Waypoint Map

The controller uses the Levine track centerline waypoint file:

```text
maps/levine_centerline.csv
```

The CSV file contains the track centerline coordinates and track boundary information.

The default waypoint path in the current development workspace is:

```text
~/controller_ws/src/technocrats_controller/maps/levine_centerline.csv
```

If the package is installed in a different location, update the `waypoint_file` parameter accordingly.

## 10. Development Notes

- ROS 2 distribution: Jazzy
- Controller package: `technocrats_controller`
- Executable: `controller`
- Node: `technocrats_controller`
- Team: Technocrats
- Competition: KFUPM F1TENTH Autonomous Racing Competition

## 12. Team

**Team Technocrats**
KFUPM F1TENTH Autonomous Racing Competition
King Fahd University of Petroleum & Minerals (KFUPM)

---

*Developed for autonomous racing research and experimentation using the F1TENTH platform.*
