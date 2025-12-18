# Docker-RosWS

ROS 2 Humble workspace for the autonomous vehicle stack.

## Quick start (after ROS 2 Humble exists on your machine)
```bash
colcon build
source install/setup.bash
ros2 launch bringup system.launch.py
```

## Docker Quick Start

To build and run the workspace using Docker:

1. Build the Docker image:
   ```bash
   docker build -t ros2_workspace docker/
   ```

2. Run the Docker container:
   ```bash
   docker run -it --rm ros2_workspace
   ```

This ensures all dependencies are encapsulated within the Docker container.

## Repository layout

- `src/interfaces`: interface-only packages (msgs/srvs/actions)
- `src/bringup`: top-level launch entrypoints
- `src/description`: robot description + TF conventions
- `src/config`: YAML parameters, calibration, maps, visualization configs
- `src/drivers`: hardware interface packages
- `src/perception`: perception packages
- `src/localization`: localization packages
- `src/planning`: planning packages
- `src/control`: control packages
- `src/tools`: developer utilities and diagnostics
