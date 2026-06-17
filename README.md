Autonomous Mobile Robot Vacuum Cleaner



Overview
This project presents an Autonomous Mobile Robot Vacuum Cleaner developed as a graduation project. The robot is capable of:

Autonomous SLAM mapping

Self-localization using AMCL

Autonomous navigation using Navigation2

Frontier-based exploration

Coverage path planning for systematic cleaning

Real-time obstacle avoidance

EKF sensor fusion

Differential drive motion control

The system is built using ROS2 Humble running on a Raspberry Pi 4 and integrates LiDAR, IMU, wheel encoders, and Arduino-based motor control.

System Architecture
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
Hardware Components
Component	Description
Raspberry Pi 4B (4GB)	Main ROS2 controller
RPLidar A1	360° LiDAR sensor
Arduino UNO	Low-level motor control
MPU6050	IMU sensor
DC Motors with Encoders	Differential drive
L298N	Motor driver
Suction Motor	Vacuum cleaning
Caster Wheel	Passive support wheel
Software Features
SLAM Mapping
Using SLAM Toolbox to generate occupancy grid maps from LiDAR and odometry.

Localization
AMCL particle filter estimates robot pose within a pre-built map.

Navigation
Navigation2 provides:

Global path planning

Local path planning

Obstacle avoidance

Costmap generation

Frontier Exploration
Automatically discovers unexplored areas and generates exploration goals.

Coverage Planner
Generates zig-zag cleaning paths to ensure full floor coverage while minimizing overlap.

Sensor Fusion
Extended Kalman Filter (EKF) fuses:

Encoder odometry

IMU measurements

for improved localization accuracy.

Package Structure
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
Simulation
Launch Gazebo:

ros2 launch my_bot launch_sim.launch.py
Run SLAM:

ros2 launch my_bot online_async_launch.py
Run Localization:

ros2 launch my_bot localization_launch.py
Run Navigation:

ros2 launch my_bot navigation_launch.py
Results
Autonomous map generation

Accurate localization using AMCL

Obstacle avoidance with Nav2

Autonomous exploration of unknown environments

Full coverage cleaning using zig-zag planning

Successful hardware-software integration

Future Improvements
Auto docking and charging

Battery monitoring system

MOSFET motor drivers

AI-based dynamic obstacle avoidance

ROS2 cloud monitoring

Team
Graduation Project 2025/2026

Faculty of Engineering
Department of Mechatronics & Mechanical Engineering

This README will look much more professional to examiners, recruiters, and anyone visiting your GitHub repository than a generic ROS package README.
