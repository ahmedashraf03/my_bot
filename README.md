# 🤖 Autonomous Mobile Robot Vacuum Cleaner

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Raspberry%20Pi%204-4GB-red)
![Navigation](https://img.shields.io/badge/Nav2-Enabled-green)

## 📖 Overview

This project presents an autonomous mobile robot vacuum cleaner capable of:

- Autonomous SLAM Mapping
- Localization using AMCL
- Navigation using Nav2
- Frontier Exploration
- Coverage Path Planning
- EKF Sensor Fusion
- Obstacle Avoidance

---

## 🏗️ System Architecture
```text
RPLidar A1
IMU MPU6050
Wheel Encoders
      │
      ▼
     ROS2
      │
 ┌─────────────┐
 │ SLAM Toolbox│
 └─────────────┘
      │
 ┌─────────────┐
 │    AMCL     │
 └─────────────┘
      │
 ┌─────────────┐
 │ Navigation2 │
 └─────────────┘
      │
 ┌─────────────┐
 │ Frontier    │
 │ Explorer    │
 └─────────────┘
      │
 ┌─────────────┐
 │ Coverage    │
 │ Planner     │
 └─────────────┘
      │
  cmd_vel
      │
diffdrive_arduino
      │
 Arduino UNO
      │
   L298N
      │
 DC Motors
```
## 🔧Hardware Components

```text
| Component               | Description             |
| ----------------------- | ----------------------- |
| Raspberry Pi 4B (4GB)   | Main ROS2 controller    |
| RPLidar A1              | 360° LiDAR sensor       |
| Arduino UNO             | Low-level motor control |
| MPU6050                 | IMU sensor              |
| DC Motors with Encoders | Differential drive      |
| L298N                   | Motor driver            |
| Suction Motor           | Vacuum cleaning         |
| Caster Wheel            | Passive support wheel   |
```
---
## Software Features
## SLAM Mapping
Using SLAM Toolbox to generate occupancy grid maps from LiDAR and odometry.

## Localization
AMCL particle filter estimates robot pose within a pre-built map.]

## Navigation
Navigation2 provides:
  Global path planning
  Local path planning
  Obstacle avoidance
  Costmap generation

## Frontier Exploration
Automatically discovers unexplored areas and generates exploration goals.

## Coverage Planner
Generates zig-zag cleaning paths to ensure full floor coverage while minimizing overlap.

## Sensor Fusion
Extended Kalman Filter (EKF) fuses: 
  Encoder odometry
  IMU measurements
for improved localization accuracy.

---
## Package Structure

```text
my_bot/
├── config/          # Nav2, SLAM, EKF and controller parameters
├── description/     # URDF/Xacro robot model
├── launch/          # ROS2 launch files
├── scripts/
│   ├── frontier_explorer.py
│   └── coverage_planner.py
├── worlds/          # Gazebo simulation worlds
├── package.xml
└── README.md
```
## Simulation
## Launch Gazebo:
ros2 launch my_bot launch_sim.launch.py

## Run SLAM:
ros2 launch my_bot online_async_launch.py

## Run Localization:
ros2 launch my_bot localization_launch.py

## Run Navigation:
ros2 launch my_bot navigation_launch.py

## Run Frontier Explorer
ros2 run my_bot frontier_explorer.py

## Run Coverage Planner
ros2 run my_bot coverage_planner.py

## Results
Autonomous map generation
Accurate localization using AMCL
Obstacle avoidance with Nav2
Autonomous exploration of unknown environments
Full coverage cleaning using zig-zag planning
Successful hardware-software integration


