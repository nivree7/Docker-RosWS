# my_robot_localization package

Hosts sensor fusion & localization nodes (& their parameter files) that are responsible for estimating the robots state.

**Build Type:** `ament_python`

**Environment:** Physical & Simulation

## System Interfaces
### Subscription Topics
* **`/imu/data`** (`sensor_msgs/Imu`): Absolute orientation and acceleration from base pkg.
* **`/gps/fix`** (`sensor_msgs/NavSatFix`): Global positioning data from base pkg.
* **`/diff_cont/odom`** (`nav_msgs/Odometry`): Wheel encoder odometry.
### Publishing Topics
* **`/odometry/global`** (`nav_msgs/Odometry`): The globally filtered odometry state.
* **`/odometry/local`** (`nav_msgs/Odometry`): The locally filtered odometry state.
* **`/gps/filtered`** (`sensor_msgs/NavSatFix`): The filtered GPS coordinates.
### TF Transforms Published
* **`map`** -> **`odom`** (Published by `ekf_filter_node_map`)
* **`odom`** -> **`base_footprint`** (Published by `ekf_filter_node_odom`)

## Package Structure

### Launch Files
* `gps_localization.launch.py`
    * Launches the `navsat_transform_node`, `ekf_filter_node_map`, and `ekf_filter_node_odom`.


### Config Files
* `ekf_config.yaml`
    * Defines the parameters, topic subscriptions, and covariance matrices for the EKFs and NavSat Transform.

### Source Files

### Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)

## Executables & Nodes

### Nodes

### Executables