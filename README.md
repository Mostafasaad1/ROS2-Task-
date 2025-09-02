# ROS 2 Evaluation Task - Gazebo Utilities Package

This ROS 2 package provides utilities for interacting with Gazebo simulation, including model spawning, deletion, and camera image capture functionality.

## Package Overview

This package contains several nodes and a shared library for working with Gazebo simulation:

- **Gazebo Utilities Client Library**: A C++ shared library that provides core functionality for spawning and deleting models in Gazebo
- **Multiple Demo Nodes**: Various demonstration nodes showing different capabilities
- **Model Assets**: Several battery models and a factory world environment

## Nodes Description

### 1. Battery Spawner Node (`battery_spawner_node`)
A basic node that spawns battery models at random positions without camera functionality.

**Features:**
- Spawns battery models from the `models/` directory
- Random positions within specified ranges (x: [-0.21, 0.21], y: [-0.43, 0.43], z: 1.1)
- Automatic model deletion before respawning
- 5-second timer cycle

**Models used:**
- battery_9v_leader
- battery_energizer
- battery_varita
- lipo_battery

### 2. Camera Model Spawner Node (`camera_model_spawner_node`)
An enhanced version that includes camera image capture functionality.

**Additional features:**
- Subscribes to `/camera/image_raw` topic
- Captures and saves images after each model spawn
- Saves images as `image_<ID>.png` in the current directory
- All the functionality of the basic spawner node

## Prerequisites

- ROS 2 Humble Hawksbill
- Gazebo (with ROS 2 integration)
- OpenCV
- cv_bridge

** I used wsl ubuntu 22.04 

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

2. Install dependencies:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-image-transport
```

3. Build the package:
```bash
cd ~/ros2_ws
VERBOSE=1 MAKEFLAGS=-j8 colcon build --cmake-clean-first --symlink-install --event-handlers console_direct+ --executor sequential --packages-select ros2_eval_task
```

4. Source the workspace:
```bash
source install/setup.bash
```

```zsh
source install/setup.zsh # im using Zsh 
```

## Usage

### 1. Start Gazebo Simulation

First, launch Gazebo with the factory world:
```bash
ros2 launch ros2_eval_task gazebo.launch.py
```

### 2. Run the Basic Spawner Node

For basic model spawning without camera functionality:
```bash
ros2 run ros2_eval_task battery_spawner_node
```

### 3. Run the Camera-Enabled Spawner Node

For model spawning with image capture (requires a camera in the simulation):
```bash
ros2 run ros2_eval_task camera_model_spawner_node
```

### 4. Additional Demo Nodes

The package also includes demonstration nodes for testing the library:
```bash
# Simple demo with a box model
ros2 run ros2_eval_task demo_gazebo_utils_client

```

## File Structure

```
ros2_eval_task/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   ├── gazebo_utils_client.cpp      # Shared library implementation
│   ├── demo_gazebo_utils_client.cpp # Basic demo
│   ├── battery_spawner_node.cpp     # Basic spawner node
│   ├── camera_model_spawner_node.cpp # Camera-enabled spawner node
│   └── model_utils.cpp              # Model loading utilities
├── include/
│   └── ros2_eval_task/
│       ├── gazebo_utils_client.hpp  # Library header
│       └── model_utils.hpp          # Model utilities header
├── launch/
│   └── gazebo.launch.py             # Gazebo launch file
├── models/                          # Battery models
│   ├── battery_9v_leader/
│   ├── battery_energizer/
│   ├── battery_varita/
│   └── lipo_battery/
├── rviz/
│   └── default.rviz                 # RViz configuration
├── worlds/
│   └── factory.world               # Gazebo world
└── ros2_eval_task/
    └── __init__.py                 # Python package init
```
## License

This project is licensed under the [MIT License](LICENSE).
