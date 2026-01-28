# my_robot_description package

Stores robots URDF/Xacro, meshes, RViz config, and launches `robot_state_publisher` to provide the TF tree.

**Build Type:** `ament_cmake`

**Environment:** Physical & Simulation

> **Note:** Src and include folder currently removed (no C++ executables or libraries.)

## Package Structure

### Launch Files
* `rsp.launch.py`
    * Proiding base functionality

### Config Files
* `my_robot.rviz`
    * Placeholder
    * (`robot_state_publisher` + `joint_state_publisher`, sets `robot_description`)

### Source Files

### Simple Auxiliary Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)
* `xacro/`
    * `camera.xacro`
    * `gazebo_control.xacro`
    * `gps.xacro`
    * `imu.xacro`
    * `inertial_macros.xacro`
    * `lidar_3d.xacro`
    * `lidar.xacro`
    * `robot_core.xacro`
    * `robot.urdf.xacro`
    * `ros2_control.xacro`

## Executables & Nodes

### Nodes
* `<example_node>`
    * **Source File:** `<example/source.file>`
    * **Desc:** 

### Executables
