# Camera-Gated LiDAR Clustering

**Camera-gated Euclidean clustering for LiDAR object detection with camera validation and classification.**

This package implements sensor fusion between LiDAR (Ouster) and stereo camera (ZED) for robust object detection and classification. The camera "gates" (validates/filters) LiDAR clusters and provides semantic labels.

## Features

- ✅ **Multiple Gating Modes**: Hard, soft, confidence-based, distance-adaptive
- ✅ **Flexible Association**: IoU, centroid, 3D distance, or hybrid matching
- ✅ **Configurable Parameters**: All thresholds externalized to YAML
- ✅ **Placeholder Calibration**: Develop without final hardware setup
- ✅ **RViz Visualization**: 3D markers and debug image overlays
- ✅ **Modular Architecture**: Easy to test and extend

## Quick Start

### 1. Build the Packages

```bash
cd ~/ros2_ws
colcon build --packages-select fusion_msgs camera_gated_clustering
source install/setup.bash
```

### 2. Launch Sensor Fusion

```bash
# Full system (all sensors + fusion + visualization)
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py

# With options
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py \
    sensor_hostname:=os-992209000435.local \
    camera_model:=zed2 \
    gating_mode:=soft \
    enable_viz:=true

# PCL KD-tree Euclidean clustering (enabled by default)
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py \
  use_pcl_clustering:=true

# Step 2: LiDAR Detection3DArray + camera fusion node
ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py

# Optional: launch ZED from the same file (default true)
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   launch_zed:=false
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   camera_model:=zed

# Optional: enable/disable RViz overlays
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   enable_viz:=true

# Optional: disable static TF or provide a custom calibration file
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   enable_static_tf:=false
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   calibration_file:=/path/to/camera_lidar_calibration.yaml

# Optional: disable base_link -> lidar static TF
# ros2 launch camera_gated_clustering lidar_camera_fusion.launch.py \
#   enable_base_tf:=false
```

### 3. View Results in RViz

The launch file automatically opens RViz with fusion visualization:
- **Green boxes**: LiDAR-only clusters
- **Yellow boxes**: Camera-validated clusters
- **Blue boxes**: Camera detections
- **Debug image**: Overlay showing associations

## Package Structure

```
camera_gated_clustering/
├── camera_gated_clustering/         # Python modules
│   ├── __init__.py
│   ├── camera_gated_clustering_node.py  # Main fusion node
│   ├── projection_utils.py          # 3D->2D projection
│   ├── cluster_association.py       # Cluster-detection matching
│   ├── gating_strategies.py         # Filtering strategies
│   └── visualization.py             # RViz markers and debug images
├── config/
│   ├── camera_lidar_calibration.yaml  # Sensor transforms (⚠️ UPDATE THIS)
│   ├── fusion_params.yaml           # Algorithm parameters
│   └── sensor_fusion.rviz           # RViz configuration
├── launch/
│   ├── sensor_fusion_full.launch.py # Master launch file
│   └── components/                  # Modular launch components
├── docs/
│   └── CALIBRATION_GUIDE.md         # How to calibrate sensors
└── test/                            # Unit and integration tests
```

## Configuration

### Calibration (IMPORTANT!)

**Current Status**: Using placeholder transforms

The file `config/camera_lidar_calibration.yaml` contains the spatial relationship between sensors. Current values are estimates for development.

**To update calibration when hardware is mounted:**

1. Measure actual sensor positions
2. Edit `config/camera_lidar_calibration.yaml`
3. Restart launch file
4. **No code recompilation needed!**

See `docs/CALIBRATION_GUIDE.md` for detailed instructions.

### Algorithm Parameters

Edit `config/fusion_params.yaml` to tune:

**Gating Mode**:
- `soft`: Keep all clusters, add camera labels (default)
- `hard`: Only keep camera-validated clusters
- `confidence`: Gate based on fusion confidence
- `distance_adaptive`: Near=camera, far=LiDAR

**Association Method**:
- `iou`: 2D bounding box overlap
- `centroid`: Distance in image space
- `distance_3d`: 3D Euclidean distance
- `hybrid`: Weighted combination (default)

**Thresholds**:
```yaml
association:
  iou_threshold: 0.25
  max_centroid_distance_px: 100
  max_spatial_distance_m: 2.5

confidence:
  min_camera_confidence: 0.45
  min_lidar_confidence: 0.35
  camera_weight: 0.6  # Trust camera more for classification
  lidar_weight: 0.4   # Trust LiDAR more for geometry
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ouster/points` | sensor_msgs/PointCloud2 | LiDAR point cloud |
| `/zed/zed_node/left/image_rect_color` | sensor_msgs/Image | Camera image |
| `/zed/zed_node/left/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |
| `/zed/zed_node/obj_det/objects` | zed_msgs/ObjectsStamped | Camera detections |

### Step 2 (Detection3DArray Fusion) Topics

The step 2 node (`lidar_camera_fusion_node.py`) projects LiDAR 3D detections into the camera image and attaches a lightweight label.

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/detections3d` | vision_msgs/Detection3DArray | LiDAR cluster boxes from step 1 |
| `/zed/zed_node/left/image_rect_color` | sensor_msgs/Image | Camera image for ROI classification |
| `/zed/zed_node/left/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

### Step 2 Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fusion/detections3d` | vision_msgs/Detection3DArray | LiDAR detections with stub labels |
| `/fusion/debug_image` | sensor_msgs/Image | Image with projected ROIs and labels |
| `/fusion/markers` | visualization_msgs/MarkerArray | 3D overlay boxes and labels for RViz |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fusion/enriched_clusters` | fusion_msgs/EnrichedClusterArray | Fused clusters with labels |
| `/fusion/markers` | visualization_msgs/MarkerArray | 3D bounding boxes |
| `/fusion/debug_image` | sensor_msgs/Image | Overlay visualization |
| `/fusion/filtered_cloud` | sensor_msgs/PointCloud2 | Filtered point cloud |

## Custom Messages

### `EnrichedCluster.msg`

```
std_msgs/Header header
geometry_msgs/Point centroid          # 3D position
geometry_msgs/Vector3 dimensions      # Bounding box size
string object_class                   # "person", "vehicle", etc.
float32 camera_confidence             # [0-1]
bool camera_validated                 # Validated by camera?
int32 point_count                     # Number of LiDAR points
float32 lidar_confidence              # [0-1]
float32 fusion_confidence             # Combined score [0-1]
int32 cluster_id                      # Unique ID
```

## Dependencies

### ROS2 Packages
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`
- `tf2_ros`, `message_filters`, `cv_bridge`
- `fusion_msgs` (custom messages)
- Optional: `zed_msgs` (for ZED camera)

### Python Packages
- `numpy`, `opencv-python`, `scipy`

Install Python dependencies:
```bash
sudo apt install python3-numpy python3-opencv python3-scipy
```

## Testing Without Hardware

### Mock Sensor Publisher

```bash
# Run mock data generator (for development)
ros2 run camera_gated_clustering mock_sensor_publisher.py
```

### Unit Tests

```bash
colcon test --packages-select camera_gated_clustering
colcon test-result --verbose
```

## Tuning Guide

### 1. Association is Poor

**Symptoms**: Clusters not matching camera detections

**Solutions**:
- Check calibration in RViz TF view
- Increase `iou_threshold` (make less strict)
- Increase `max_centroid_distance_px`
- Try different `association.method`

### 2. Too Many False Positives

**Symptoms**: Detecting objects that aren't there

**Solutions**:
- Use `hard` gating mode
- Increase `min_camera_confidence`
- Increase `min_lidar_confidence`
- Reduce `clustering.max_cluster_size`

### 3. Missing Detections

**Symptoms**: Real objects not detected

**Solutions**:
- Use `soft` gating mode
- Decrease confidence thresholds
- Check LiDAR ROI settings
- Verify clustering parameters

### 4. Poor Performance

**Symptoms**: High CPU usage, low frame rate

**Solutions**:
- Increase `voxel_leaf_size` (more downsampling)
- Reduce ROI bounds
- Disable debug image publishing
- Use simpler association method

## Advanced Features

### Distance-Adaptive Gating

Automatically adjusts gating based on object distance:
```yaml
gating:
  mode: "distance_adaptive"

confidence:
  near_threshold_m: 5.0   # Close objects: require camera
  far_threshold_m: 15.0   # Far objects: rely on LiDAR
```

### Dynamic Reconfigure

Change parameters at runtime:
```bash
ros2 param set /camera_gated_clustering_node gating.mode hard
ros2 param set /camera_gated_clustering_node confidence.min_camera_confidence 0.6
```

## Troubleshooting

### "Waiting for camera info..."

**Cause**: Camera not publishing CameraInfo

**Fix**:
```bash
# Check camera topics
ros2 topic list | grep camera_info
ros2 topic echo /zed/zed_node/left/camera_info
```

### "TF lookup failed"

**Cause**: Missing transform between sensors

**Fix**:
```bash
# View TF tree
ros2 run tf2_tools view_frames
# Check if os_sensor -> zed_camera_center exists

# Verify static transform publisher is running
ros2 node list | grep static_transform
```

### No associations found

**Cause**: Calibration incorrect or sensors not aligned

**Fix**:
1. View debug image: `ros2 run rqt_image_view rqt_image_view /fusion/debug_image`
2. Check if LiDAR projections appear on camera image
3. Adjust calibration in `camera_lidar_calibration.yaml`

## Development

### Adding New Gating Strategy

1. Create new class in `gating_strategies.py` inheriting `GatingStrategy`
2. Implement `apply()` method
3. Register in `GatingFactory`
4. Add to config options

### Adding New Association Method

1. Add method to `ClusterAssociator` class
2. Implement matching logic
3. Update config to include new method

## License

MIT License - See LICENSE file

## Contributors

- Jeff Chang (@JEFFCHANG0501)

## Citation

If you use this work, please cite:
```bibtex
@software{camera_gated_clustering,
  title={Camera-Gated LiDAR Clustering},
  author={Chang, Jeff},
  year={2026},
  url={https://github.com/JEFFCHANG0501/lidar_cluster_detection}
}
```

## References

- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [Ouster ROS2 Driver](https://github.com/ouster-lidar/ouster-ros)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
