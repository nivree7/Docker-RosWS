# Camera-Gated Clustering - Quick Reference

## 📦 What Was Created

### Packages (2)
1. **fusion_msgs** - Custom ROS2 messages
2. **camera_gated_clustering** - Main Python fusion package

### Files Created (21)

#### fusion_msgs/
- ✅ msg/EnrichedCluster.msg
- ✅ msg/EnrichedClusterArray.msg
- ✅ msg/AssociationDebug.msg
- ✅ CMakeLists.txt
- ✅ package.xml

#### camera_gated_clustering/
**Python Modules (6):**
- ✅ camera_gated_clustering/__init__.py
- ✅ camera_gated_clustering/projection_utils.py
- ✅ camera_gated_clustering/cluster_association.py
- ✅ camera_gated_clustering/gating_strategies.py
- ✅ camera_gated_clustering/visualization.py
- ✅ camera_gated_clustering/camera_gated_clustering_node.py

**Config Files (2):**
- ✅ config/camera_lidar_calibration.yaml (⚠️ PLACEHOLDER - Update later!)
- ✅ config/fusion_params.yaml

**Launch Files (1):**
- ✅ launch/sensor_fusion_full.launch.py

**Documentation (3):**
- ✅ README.md
- ✅ docs/CALIBRATION_GUIDE.md
- ✅ IMPLEMENTATION_SUMMARY.md

**Build Files (4):**
- ✅ setup.py
- ✅ setup.cfg
- ✅ package.xml
- ✅ CMakeLists.txt
- ✅ resource/camera_gated_clustering

**Scripts (1):**
- ✅ build_fusion.sh (executable)

---

## 🚀 Build Instructions

```bash
# Option 1: Use build script
cd ~/ros2_ws/src/camera_gated_clustering
./build_fusion.sh

# Option 2: Manual build
cd ~/ros2_ws
colcon build --packages-select fusion_msgs camera_gated_clustering --symlink-install
source install/setup.bash
```

---

## 🎮 Launch Commands

### Basic Launch
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py
```

### With Custom Parameters
```bash
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py \
    sensor_hostname:=os-992209000435.local \
    camera_model:=zed2 \
    gating_mode:=soft \
    enable_viz:=true
```

### Available Gating Modes
- `soft` - Keep all clusters, add labels (default)
- `hard` - Only camera-validated clusters
- `confidence` - Threshold-based filtering
- `distance_adaptive` - Adaptive by distance

---

## 📊 Topics

### Subscribed
- `/ouster/points` - LiDAR point cloud
- `/zed/zed_node/left/image_rect_color` - Camera image
- `/zed/zed_node/left/camera_info` - Camera intrinsics
- `/zed/zed_node/obj_det/objects` - Camera detections

### Published
- `/fusion/enriched_clusters` - Fused results
- `/fusion/markers` - RViz 3D markers
- `/fusion/debug_image` - Debug visualization
- `/fusion/filtered_cloud` - Filtered point cloud

---

## ⚙️ Configuration Files

### 1. Calibration (UPDATE THIS!)
**File:** `config/camera_lidar_calibration.yaml`

```yaml
camera_lidar_transform:
  translation:
    x: 0.0  # ⚠️ Measure and update!
    y: 0.0
    z: 0.1
  rotation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

### 2. Parameters (Tune These)
**File:** `config/fusion_params.yaml`

Key settings:
- `gating.mode` - Filtering strategy
- `association.method` - Matching algorithm
- `association.iou_threshold` - Min overlap (0.25)
- `confidence.min_camera_confidence` - Min camera score (0.45)
- `confidence.min_lidar_confidence` - Min LiDAR score (0.35)

---

## 🔍 Debugging

### View Debug Image
```bash
ros2 run rqt_image_view rqt_image_view /fusion/debug_image
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
```

### Monitor Clusters
```bash
ros2 topic echo /fusion/enriched_clusters
```

### View Logs
```bash
ros2 topic echo /rosout | grep camera_gated
```

---

## 🛠️ Common Tasks

### Update Calibration
1. Edit `config/camera_lidar_calibration.yaml`
2. Restart launch file
3. No rebuild needed!

### Change Gating Mode
```bash
# During runtime
ros2 param set /camera_gated_clustering_node gating.mode hard

# Or in launch
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py gating_mode:=hard
```

### Tune Association
Edit `config/fusion_params.yaml`:
```yaml
association:
  iou_threshold: 0.30  # Increase to be less strict
  max_centroid_distance_px: 150
```

### Disable Visualization
```bash
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py enable_viz:=false
```

---

## ⚠️ Important Notes

1. **Calibration is placeholder!** 
   - Current values: [0, 0, 0.1] translation, [0, 0, 0] rotation
   - Update when hardware is mounted
   - See `docs/CALIBRATION_GUIDE.md`

2. **ZED object detection needed**
   - Must enable in ZED config
   - Topic: `/zed/zed_node/obj_det/objects`
   - Currently returns empty (to be integrated)

3. **Clustering placeholder**
   - Simple clustering implemented
   - Can integrate existing Euclidean clustering
   - Designed to be swappable

4. **Python dependencies**
   ```bash
   sudo apt install python3-numpy python3-opencv python3-scipy
   ```

---

## 📖 Documentation

- **User Guide**: `README.md`
- **Calibration**: `docs/CALIBRATION_GUIDE.md`
- **Implementation**: `IMPLEMENTATION_SUMMARY.md`

---

## ✅ Testing Checklist

Before hardware integration:
- [ ] Build succeeds without errors
- [ ] Messages defined correctly
- [ ] Parameters load from YAML
- [ ] Node starts without crashes
- [ ] TF frames published
- [ ] Review code and configs

After hardware mounted:
- [ ] Measure and update calibration
- [ ] Verify TF tree
- [ ] Test with live sensors
- [ ] Check debug image alignment
- [ ] Tune parameters
- [ ] Validate associations

---

## 🎯 Architecture Summary

```
Sensor Data → Synchronization → Association → Gating → Publishing
     ↓              ↓               ↓            ↓          ↓
  LiDAR          Message         Matching    Filtering  Enriched
  Camera         Filters        Algorithms  Strategies  Clusters
                                     ↓
                               Projection
                                 Utils
```

**Key Components:**
- **ProjectionUtils**: 3D→2D camera projection
- **ClusterAssociator**: Cluster-detection matching
- **GatingStrategy**: Validation and filtering
- **FusionVisualizer**: RViz and debug visualization
- **Main Node**: Orchestration and ROS2 interface

---

## 🚨 Troubleshooting Quick Fixes

**"No module named 'camera_gated_clustering'"**
```bash
source ~/ros2_ws/install/setup.bash
```

**"TF lookup failed"**
- Check static_transform_publisher is running
- Verify frame IDs match config

**"Waiting for camera info"**
- Check ZED camera is running
- Verify topic: `ros2 topic list | grep camera_info`

**No associations**
- Check calibration alignment in debug image
- Increase association thresholds
- Verify both sensors publishing data

---

## 🎓 Learning Resources

1. **Understanding the code**:
   - Start with `camera_gated_clustering_node.py` (main logic)
   - Review `projection_utils.py` (coordinate transforms)
   - Study `cluster_association.py` (matching algorithms)

2. **Parameter tuning**:
   - Read `fusion_params.yaml` comments
   - Experiment with different modes
   - Use debug image for feedback

3. **Calibration**:
   - Follow `docs/CALIBRATION_GUIDE.md` step-by-step
   - Use visual tuning method first
   - Refine with measurements

---

## 📞 Support

If you encounter issues:
1. Check documentation (README, CALIBRATION_GUIDE)
2. Review logs: `ros2 topic echo /rosout`
3. Verify prerequisites (dependencies, sensor drivers)
4. Test components individually
5. Ask with specific error messages and configs

---

## 🎉 Success Indicators

You'll know it's working when:
- ✅ Nodes start without errors
- ✅ TF tree is complete
- ✅ Debug image shows aligned projections
- ✅ Yellow association lines appear
- ✅ Enriched clusters published with labels
- ✅ RViz shows colored bounding boxes
- ✅ High camera_validated_count

---

**Ready to build? Run:** `./build_fusion.sh`

**Questions? Read:** `README.md` and `docs/CALIBRATION_GUIDE.md`

**Happy Fusing! 🚀📷🔬**
