# Repo Discovery Notes (Sensor Fusion)

## Packages Found

From `~/ros2_ws/src`:

- **LiDAR clustering**: `lidar_cluster_detection/lidar_euclidean_clustering`
- **LiDAR publisher (PCD)**: `lidar_cluster_detection/pcd_publisher`
- **Ouster driver**: `ouster-ros/ouster-ros`
- **ZED camera**: `zed-ros2-wrapper/zed_wrapper`

## Existing LiDAR Topics

From `lidar_euclidean_clustering/src/euclidean_clustering.cpp`:

- Subscribed: `/vls128_0/velodyne_points`
- Published:
  - `/vls128_0/filter_cloud`
  - `/vls128_0/object/markers` (MarkerArray)
  - `/eaglecar` (Marker)

From `lidar_euclidean_clustering/launch/ouster_clustering.launch.py`:

- Remaps:
  - `/vls128_0/velodyne_points` → `/ouster/points`
  - `/vls128_0/filter_cloud` → `/ouster/filter_cloud`
  - `/vls128_0/object/markers` → `/ouster/clusters`

From `pcd_publisher/src/pcd_publisher.cpp`:

- Published: `/vls128_0/velodyne_points`

## Existing LiDAR Frames

From `ouster_clustering.launch.py`:

- `sensor_frame`: `os_sensor`
- `lidar_frame`: `os_lidar`
- `imu_frame`: `os_imu`

From `pcd_publisher.cpp`:

- PointCloud2 `frame_id`: `vls_128`

From `lidar_euclidean_clustering/config/ouster_clustering.rviz`:

- RViz Fixed Frame: `os_sensor`

From `ouster_live_clustering.launch.py`:

- Static TF: `base_link` → `vls_128`

## Existing Camera Topics (ZED)

ZED topics are not explicitly defined in the launch file; the ZED wrapper constructs topics based on `camera_name` and `namespace`.

To discover actual runtime topics:

```bash
ros2 topic list | grep zed
ros2 topic list | grep camera_info
```

## Existing Launch Files

LiDAR:

- `lidar_euclidean_clustering/launch/ouster_clustering.launch.py`
  - Ouster driver + Euclidean clustering + RViz
- `lidar_euclidean_clustering/launch/ouster_live_clustering.launch.py`
  - Ouster driver + clustering + static TF + RViz

PCD Publisher:

- `pcd_publisher/launch/pcd_publisher_node.launch.py`

Camera:

- `zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py`

## Existing RViz Configs

- `lidar_euclidean_clustering/config/ouster_clustering.rviz`
  - Shows `/ouster/points`, `/ouster/filter_cloud`, `/ouster/clusters`
- `pcd_publisher/config/config.rviz`

## Existing TF Publishers

From `ouster_live_clustering.launch.py`:

- `static_transform_publisher`: `base_link` → `vls_128`

## How to Run Sensor Bring-up + Clustering

### Ouster + Clustering + RViz

```bash
ros2 launch lidar_euclidean_clustering ouster_clustering.launch.py \
  sensor_hostname:=os-992209000435.local \
  viz:=true
```

### Ouster + Clustering (alternate launch)

```bash
ros2 launch lidar_euclidean_clustering ouster_live_clustering.launch.py \
  viz:=true
```

### ZED Camera

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### PCD Publisher

```bash
ros2 launch pcd_publisher pcd_publisher_node.launch.py
```

## Notes / Gaps

- LiDAR clustering currently publishes MarkerArray only; no standardized 3D detection message yet.
- ZED topic names must be verified at runtime (not hard-coded in launch file).
- TF chain between LiDAR and camera is not defined in this repo (placeholder TF required).
