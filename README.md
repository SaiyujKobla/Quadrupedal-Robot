# Quadrupedal Robot

A custom quadrupedal robotics platform integrating inverse kinematics, gait generation, balance control, computer vision, LiDAR/ToF sensor fusion, and autonomous obstacle avoidance.

## Overview

This project is a custom-built quadrupedal robot designed to autonomously perceive and navigate its environment. The system combines low-level locomotion on an ESP32 with higher-level perception and decision-making on a computer.

The robot uses:

- Custom inverse kinematics and gait-generation code for leg control
- Balance and orientation feedback from an IMU
- An ESP32-CAM for computer vision
- YOLO object detection and tracking
- RPLIDAR A1 distance measurements
- Time-of-Flight (ToF) sensing
- Ultrasonic sensing support
- Sensor fusion and obstacle-danger scoring
- UDP communication between the robot and the computer
- Autonomous obstacle-avoidance decision logic

The long-term goal is to combine these components into a fully autonomous quadrupedal platform capable of interpreting its surroundings and translating sensor data into stable physical movement.

## System Architecture

The project is divided into two main systems.

### Robot / ESP32

The ESP32 handles time-sensitive hardware control, including:

- Servo control
- Inverse kinematics
- Walking gait generation
- Sit-to-stand movement
- Balance control
- IMU orientation sensing
- LiDAR and ToF sensor acquisition
- Communication with the computer

### Computer / Python

The computer handles higher-level perception and decision-making, including:

- ESP32-CAM video reception
- YOLO object detection and tracking
- Object memory
- Sensor fusion
- Obstacle danger scoring
- Obstacle-avoidance decisions
- Visualization
- UDP communication with the main ESP32

A simplified data flow is:

```text
ESP32-CAM ───────► Computer Vision
                        │
                        ▼
                  Object Tracking
                        │
                        ▼
RPLIDAR / ToF ──► Sensor Fusion ──► Danger Scoring
                        │
                        ▼
                 Avoidance Logic
                        │
                        ▼
                     UDP
                        │
                        ▼
                 Main ESP32
                        │
                        ▼
          Gait / Balance / Servo Control
```

## Repository Structure

```text
Quadrupedal-Robot/
├── firmware/
│   └── esp32/
│       ├── QuadrupedalRobot/
│       │   ├── BalanceController.cpp
│       │   ├── BalanceController.h
│       │   ├── GaitCycle.cpp
│       │   ├── GaitCycle.h
│       │   ├── IMUOrientation.cpp
│       │   ├── IMUOrientation.h
│       │   ├── InverseKinematics.cpp
│       │   ├── InverseKinematics.h
│       │   ├── QuadrupedalRobot.ino
│       │   ├── SitToStand.cpp
│       │   └── SitToStand.h
│       │
│       └── SensorBridge/
│           ├── SensorBridge.ino
│           └── secrets.example.h
│
├── models/
│   └── yolo11n.pt
│
├── python/
│   ├── avoidance/
│   ├── networking/
│   ├── object_detection/
│   ├── sensors/
│   ├── visualization/
│   ├── config.py
│   └── main.py
│
├── tests/
├── .gitignore
├── requirements.txt
└── README.md
```

## Main Software Components

### Inverse Kinematics

`InverseKinematics` converts desired foot positions into servo angles for each leg while accounting for the physical geometry and orientation of the robot.

### Gait Cycle

`GaitCycle` coordinates the movement of all four legs to generate walking motion.

### Sit to Stand

`SitToStand` provides controlled transitions between resting and standing positions.

### Balance Controller

`BalanceController` uses orientation feedback to help compensate for body tilt and improve stability.

### IMU Orientation

`IMUOrientation` processes inertial measurements used by the balance system.

### Sensor Bridge

`SensorBridge` collects distance-sensor readings on the ESP32 and transmits sensor packets to the computer over UDP.

### Object Detection

The Python perception pipeline uses a YOLO11 model to detect and track objects from the ESP32-CAM video stream.

### Sensor Fusion

LiDAR, ToF, and other sensor measurements are combined with camera detections to create a more complete representation of nearby obstacles.

### Avoidance Logic

Detected obstacles are divided into spatial regions, scored for danger, and used to determine an appropriate high-level avoidance command.

## Hardware

The current platform includes:

- ESP32 main controller
- ESP32-CAM
- Servo-driven quadrupedal leg system
- RPLIDAR A1
- Time-of-Flight distance sensor
- Ultrasonic distance sensor support
- IMU
- Custom 3D-printed structural and leg components
- External power regulation for the robot electronics and actuators

## Python Setup

Python 3.12 has been used for the current development environment.

From the repository root:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run the main Python system with:

```bash
PYTHONPATH=python python python/main.py
```

## Network Configuration

Computer-side networking settings are stored in:

```text
python/config.py
```

This includes settings such as:

- Main ESP32 IP address
- Sensor UDP port
- Command UDP port
- Sensor enable/disable flags
- Command-transmission enable/disable flag

During development, movement-command transmission can be disabled with:

```python
ENABLE_COMMAND_TX = False
```

This allows the perception and sensor-fusion pipeline to be tested without sending movement commands to the robot.

## ESP32 Wi-Fi Configuration

Real Wi-Fi credentials are intentionally excluded from Git.

The repository contains:

```text
firmware/esp32/SensorBridge/secrets.example.h
```

Create a local copy named:

```text
secrets.h
```

and add the local Wi-Fi credentials there.

Example:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

#endif
```

`secrets.h` is ignored by Git and should never be committed.

## Current Development Status

### Working / Implemented

- Custom inverse kinematics
- Servo calibration framework
- Sit-to-stand motion
- Walking gait generation
- IMU orientation processing
- Balance-controller framework
- ESP32-CAM video streaming
- YOLO object detection and tracking
- RPLIDAR A1 communication
- ToF distance sensing
- ESP32-to-computer UDP sensor communication
- Computer-side sensor processing and fusion
- Obstacle danger scoring
- High-level obstacle-avoidance logic

### In Development / Tuning

- Balance-controller integration with walking
- Stability tuning during locomotion
- Turning gait behavior
- Full autonomous perception-to-movement integration
- Additional robot calibration and hardware testing

## Development Notes

Many robot behaviors depend on physical calibration values specific to the current mechanical build. Servo offsets, joint limits, gait parameters, balance-controller constants, and sensor positions should therefore be changed carefully and tested incrementally.

Hardware test and calibration sketches will be organized separately from the primary robot-control firmware so that the main control code remains easy to identify.

## Project Goal

The project explores how custom locomotion, computer vision, and multi-sensor perception can be integrated into a single quadrupedal robotic system. Rather than treating walking, sensing, and obstacle avoidance as independent demonstrations, the goal is to build a platform in which environmental measurements directly inform physical behavior.
