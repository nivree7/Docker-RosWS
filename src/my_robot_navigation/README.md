# my_robot_navigation package

Contains Nav2 specific config & launch files. Provides Navigation behavior.

**Note:** Src and include folder currently removed (no C++ executables or libraries.)

**Build Type:** `ament_cmake`

**Environment:** Physical & Simulation

## Required Frames/Topics
### Frames
* `odom` (global_frame for BOTH costmaps)
* `base_link` (robot_base_frame & fused_cloud.sensor_frame) 
### Subscription Topics
* `fused_cloud` (for fused_cloud.topic, expects PointCloud2 message type)
* `odom` (controller_server.odom_topic)
### Publishing Topics
* `ackermann_cmd` (controller_server.command_topic)
>**Note:** DONT USE ACKERMANN_CMD, need to change this.
### TF Transforms
* `odom` -> `base_link` (robot pose)
* `base_link` -> `sensor frames`(URDF)

## Package Structure

### Launch Files  
* `nav2.launch.py`
    * Providing base functionality

### Config Files  
* `nav2_params.yaml`

### Source Files
> **Note:** Won't use local source files?

### Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)

## Executables & Nodes

### Nodes
> **Note:** Won't use local nodes?
* `<example_node>`
    * **Source File:** `<example/source.file>`
    * **Desc:** 

### Executables
> **Note:** Won't use local executables?