# my_robot_igvc_mission package

Contains IGVC 2026 competition specific behavior.

**Build Type:** `ament_python`

**Environment:** Physical & Simulation

## System Interfaces
### Subscription Topics
### Publishing Topics
### Services Called
* `/fromLL` (Translates raw target GPS Latitude/Longitude coordinates to local `map` X/Y metric coordinates via the `navsat_transform` node)
### Action Clients
* **Nav2 Simple Commander (`BasicNavigator`)**: Sends the translated `PoseStamped` waypoints to the Nav2 navigation stack (via the `/navigate_through_poses` action server) to drive the robot.
### Required TF Frames
* **`map`**: Target waypoints are attached to the `map` frame. The mission package relies on the localization package to broadcast the continuous `map` -> `odom` -> `base_footprint` TF tree.


## Package Structure

### Launch Files  
* `mission_launch.py` 
    * Launches the `waypoint_follower` executable and loads target parameters from config file.

### Config Files  
* `mission_config.py`  
    * Defines the `target_waypoints` parameter (a flattened array of Lat/Long pairs).


### Source Files
* `nodes/waypoint_follower.py`
    * Extracts GPS coord waypoint params, asyc service requests for coord translation, and goal execution via the `simple_commander_api`.


### Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)

## Executables & Nodes

### Nodes
* `waypoint_follower_params`
    * **Source File:** `nodes/waypoint_follower.py`
    * **Desc:** Temp helper node to extract `target_waypoints` list from params.
* `waypoint_translator`
    * **Source File:** `nodes/waypoint_follower.py`
    * **Desc:** Temp helper node to establish client connection to the `/fromLL/` service, & request coord translations.

### Executables
* `waypoint_follower`
    * **Source File:** `nodes/waypoint_follower.py`
    * **Desc:** Main executable to orchestrate entire mission sequence.