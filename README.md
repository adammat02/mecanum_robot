# Mecanum Robot

> **Work in progress** — this README will be updated as the project evolves.

## Overview

This project focuses on building a **mecanum-wheeled robot** capable of **autonomous navigation**. Mecanum wheels allow omnidirectional movement — the robot can move forward, backward, sideways, and rotate in place without changing its orientation, making it well-suited for tight spaces and complex navigation tasks.

## Goals

- Design and simulate a mecanum-drive robot in ROS 2
- Hardware integration with **ros2_control**
- Integrate sensors (e.g. LiDAR, camera) for environment perception
- Enable autonomous navigation using SLAM and path planning

## Based on

This project builds upon my previous work:  
**[RoMKoSz](https://github.com/adammat02/RoMKoSz)**

## Tech Stack

- **ROS 2** (Robot Operating System 2)
- **Gazebo** — simulation
- **URDF / Xacro** — robot description
- **Nav2** — autonomous navigation stack (planned)

## Installation

```bash
# 1. Clone into workspace
cd ~/ros2_ws/src
git clone <url> mecanum_robot

# 2. Fetch external packages from GitHub
cd ~/ros2_ws
vcs import src < src/mecanum_robot/mecanum_robot.repos

# 3. Install apt dependencies (including libserial-dev)
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install
```

> `vcs` is available via `sudo apt install python3-vcstool`.

## License

Apache-2.0
