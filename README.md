# 2D Humanoid Robot Simulation

A comprehensive 2D humanoid robot simulation framework with walking capabilities, keyboard control, and coordinate-based navigation. This project provides both standalone Python implementations and ROS 2 integration for educational and research purposes.

![Humanoid Robot Simulation](images/robot_simulation_overview.png)



## Overview
This project implements a 2D humanoid robot simulation with realistic walking gait patterns, joint articulation, and multiple control interfaces. The simulation includes standalone Python applications using Pygame and a ROS 2 package for robotics workflows.

The humanoid robot features:
- Anatomically proportioned body segments (head, torso, arms, legs, feet)
- Articulated joints with realistic movement constraints
- Walking gait with coordinated arm and leg motion
- Multiple control modalities (keyboard, mouse, coordinate input)
- Real-time inverse kinematics for natural movement



## Features
### Core Functionality
- **Realistic Walking Animation**: Sinusoidal gait patterns with proper phase relationships
- **Multi-Modal Control**: Keyboard (WASD/Arrow keys), mouse click navigation, coordinate input
- **Inverse Kinematics**: Mathematical leg positioning for natural ground contact
- **Visual Feedback**: Direction indicators, target markers, and joint visualization
- **ROS 2 Integration**: Complete package with URDF, launch files, and node implementations

### Technical Capabilities
- **Joint State Management**: 7-DOF articulated humanoid with configurable joint limits
- **Real-time Simulation**: 60 FPS rendering with smooth animation transitions
- **Path Planning**: Point-to-point navigation with obstacle-free trajectories
- **State Machine**: Idle, walking, and turning behavioral states
- **Configurable Parameters**: Adjustable robot dimensions, walking speed, and joint constraints



## System Requirements
### Operating System
- **Ubuntu 22.04 LTS (Jammy Jellyfish)** - Primary development and testing platform
- **Ubuntu 20.04 LTS (Focal Fossa)** - Supported with minor configuration changes

### Software Dependencies
#### Core Requirements
- **Python 3.8+** - Programming language runtime
- **Pygame 2.0+** - Graphics and input handling library
- **NumPy 1.19+** - Numerical computing library
- **SciPy 1.7+** - Scientific computing extensions
- **Matplotlib 3.3+** - Plotting and visualization

#### ROS 2 Requirements (Optional)
- **ROS 2 Humble Hawksbill** - Robot Operating System framework
- **robot_state_publisher** - Robot model publishing node
- **joint_state_publisher_gui** - Interactive joint control interface
- **RViz2** - 3D robot visualization tool

#### 3D Modeling (Optional)
- **OpenSCAD 2021.01+** - Parametric 3D modeling for STL generation
- **MeshLab 2020.09+** - 3D mesh processing and visualization

### Hardware Requirements
- **CPU**: Multi-core processor (2+ GHz recommended)
- **RAM**: 4 GB minimum, 8 GB recommended
- **GPU**: OpenGL 3.0+ compatible graphics adapter
- **Display**: 1024x768 minimum resolution

## Installation
### Quick Start (Standalone Python)
```bash
git clone https://github.com/yourusername/humanoid-robot-2d.git
cd humanoid-robot-2d
pip3 install -r requirements.txt
python3 src/walking_robot.py
