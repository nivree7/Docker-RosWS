# Camera-LiDAR Calibration Guide

## Current Status

⚠️ **Using placeholder calibration values**

**Current transform** (in `config/camera_lidar_calibration.yaml`):
- **Translation**: [0.0, 0.0, 0.1] meters
- **Rotation**: [0.0, 0.0, 0.0] radians

**These are ESTIMATES for development. Update when hardware is mounted!**

---

## Why Calibration Matters

Accurate calibration is essential for:
1. **Correct projection**: LiDAR points must project to correct image locations
2. **Accurate association**: Clusters and detections must align spatially
3. **Reliable fusion**: Misalignment causes false associations or missed matches

**Impact of poor calibration**:
- LiDAR projections offset from camera detections
- Low association rates
- Reduced fusion confidence
- Poor classification accuracy

---

## When Hardware is Ready

### Method 1: Manual Measurement (Quick & Simple)

**Best for**: Initial calibration, rough alignment

**Steps**:

1. **Measure translation** (distance between sensors):
   ```
   x: Forward/backward offset (positive = camera forward of LiDAR)
   y: Left/right offset (positive = camera left of LiDAR)
   z: Up/down offset (positive = camera above LiDAR)
   ```

2. **Measure rotation** (orientation difference):
   ```
   roll: Rotation around forward axis (side tilt)
   pitch: Rotation around lateral axis (up/down tilt)
   yaw: Rotation around vertical axis (left/right rotation)
   ```

3. **Update YAML file**:
   ```yaml
   camera_lidar_transform:
     translation:
       x: 0.15  # Example: camera 15cm forward
       y: 0.0
       z: 0.08  # Example: camera 8cm above
     rotation:
       roll: 0.0
       pitch: -0.087  # Example: tilted down 5 degrees
       yaw: 0.0
   ```

4. **Restart launch file**:
   ```bash
   ros2 launch camera_gated_clustering sensor_fusion_full.launch.py
   ```

5. **Verify in RViz**:
   - View TF frames
   - Check if LiDAR projections align with camera image
   - Adjust and repeat

**Tools**:
- Ruler or tape measure
- Protractor or digital level for angles
- CAD model (if available)

---

### Method 2: Checkerboard Calibration (Most Accurate)

**Best for**: Final calibration, production systems

**Requirements**:
- Large checkerboard pattern (print on poster, mount on rigid board)
- Both sensors must see checkerboard simultaneously
- Minimum 10-15 different poses/positions

**Steps**:

1. **Print checkerboard**:
   - Size: At least 1m x 1m
   - Square size: 10-15cm
   - High contrast (black/white)
   - Mount on flat, rigid surface

2. **Capture calibration data**:
   ```bash
   # Record rosbag with both sensors viewing checkerboard
   ros2 bag record /ouster/points /zed/zed_node/left/image_rect_color /zed/zed_node/left/camera_info
   
   # Move checkerboard to different positions:
   # - Different distances (2m, 5m, 10m)
   # - Different angles (left, right, up, down)
   # - Different orientations
   # Aim for 15-20 poses
   ```

3. **Run calibration tool**:
   ```bash
   # Option A: Use existing ROS calibration package
   # (You'll need to install a camera-lidar calibration package)
   
   # Option B: Use custom Python script
   python3 calibration_scripts/calibrate_camera_lidar.py --bag calibration.bag
   ```

4. **Apply calibration results**:
   - Calibration tool outputs optimized transform
   - Copy values to `camera_lidar_calibration.yaml`
   - Restart system

**Expected accuracy**: <1cm translation, <0.5° rotation

---

### Method 3: Iterative Visual Tuning (Practical)

**Best for**: Quick refinement, when measurement is difficult

**Steps**:

1. **Start with rough estimate**:
   - Use manual measurement (Method 1)
   - Or use default values if sensors are co-located

2. **Launch fusion with debug visualization**:
   ```bash
   ros2 launch camera_gated_clustering sensor_fusion_full.launch.py enable_viz:=true
   ```

3. **View debug image**:
   ```bash
   ros2 run rqt_image_view rqt_image_view /fusion/debug_image
   ```

4. **Identify misalignment**:
   - LiDAR clusters (red/green boxes) should overlap camera detections (blue boxes)
   - Note direction and magnitude of offset

5. **Adjust calibration incrementally**:
   
   **If LiDAR projects too far left**:
   ```yaml
   translation:
     y: 0.05  # Increase y (shift right)
   ```
   
   **If LiDAR projects too low**:
   ```yaml
   translation:
     z: 0.12  # Increase z (shift up)
   ```
   
   **If LiDAR projects too far**:
   ```yaml
   translation:
     x: -0.05  # Decrease x (shift back)
   ```
   
   **If LiDAR projections are tilted**:
   ```yaml
   rotation:
     pitch: 0.087  # Adjust pitch (5 degrees in radians)
   ```

6. **Restart and repeat**:
   - Make small changes (2-5cm, 2-5 degrees)
   - Test on multiple objects at different distances
   - Converge to best alignment

7. **Validate**:
   - Drive/move to different locations
   - Test with various objects
   - Check association rates: `ros2 topic echo /fusion/enriched_clusters`

---

## Testing Calibration Quality

### 1. Visualize TF Tree

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing transform tree
# Verify os_sensor -> zed_camera_center exists
```

### 2. Check Transform Values

```bash
ros2 run tf2_ros tf2_echo os_sensor zed_camera_center
# Shows current transform from LiDAR to camera
# Should match your YAML file
```

### 3. View Projection Alignment in RViz

```bash
ros2 launch camera_gated_clustering sensor_fusion_full.launch.py
# In RViz:
# - Enable "Camera" display
# - Enable "PointCloud2" display
# - View from camera perspective
# - LiDAR points should align with 3D objects
```

### 4. Check Debug Image

```bash
ros2 run rqt_image_view rqt_image_view /fusion/debug_image
```

**Good calibration indicators**:
- ✅ LiDAR projections (red/green) overlap camera detections (blue)
- ✅ Association lines (yellow) are short and straight
- ✅ Most clusters have associations (high yellow line count)

**Poor calibration indicators**:
- ❌ Systematic offset in one direction
- ❌ Few or no associations
- ❌ Long association lines (large distance between matches)

### 5. Monitor Association Metrics

```bash
ros2 topic echo /fusion/enriched_clusters
```

**Good calibration metrics**:
- High `camera_validated_count` relative to `total_clusters`
- High `association_score` values (>0.6)
- High `fusion_confidence` values

---

## Quick Update Procedure

**After hardware is mounted**:

1. Measure sensor positions
2. Edit `config/camera_lidar_calibration.yaml`:
   ```yaml
   camera_lidar_transform:
     translation:
       x: <your_measured_x>
       y: <your_measured_y>
       z: <your_measured_z>
     rotation:
       roll: <your_measured_roll>
       pitch: <your_measured_pitch>
       yaw: <your_measured_yaw>
   ```
3. Restart launch file:
   ```bash
   ros2 launch camera_gated_clustering sensor_fusion_full.launch.py
   ```
4. **No code recompilation needed!**

---

## Common Calibration Scenarios

### Scenario 1: Camera Mounted Above LiDAR

```yaml
translation:
  x: 0.0      # Centered fore/aft
  y: 0.0      # Centered laterally
  z: 0.15     # 15cm above
rotation:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### Scenario 2: Camera Forward and Above

```yaml
translation:
  x: 0.10     # 10cm forward
  y: 0.0
  z: 0.12     # 12cm above
rotation:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### Scenario 3: Camera Tilted Down 10°

```yaml
translation:
  x: 0.0
  y: 0.0
  z: 0.10
rotation:
  roll: 0.0
  pitch: -0.174   # -10 degrees in radians
  yaw: 0.0
```

### Scenario 4: Camera Offset to Left

```yaml
translation:
  x: 0.0
  y: 0.08     # 8cm to the left
  z: 0.10
rotation:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

---

## Angle Conversion Reference

| Degrees | Radians |
|---------|---------|
| 0° | 0.000 |
| 5° | 0.087 |
| 10° | 0.174 |
| 15° | 0.262 |
| 20° | 0.349 |
| 30° | 0.524 |
| 45° | 0.785 |
| 90° | 1.571 |

Formula: `radians = degrees × π / 180`

---

## Troubleshooting

### No transform published

**Symptom**: `TF lookup failed` errors

**Fix**:
1. Check if static transform publisher is running:
   ```bash
   ros2 node list | grep static_transform
   ```
2. Verify launch file includes transforms component
3. Check YAML file syntax (no tabs, proper indentation)

### Projections appear backwards

**Symptom**: LiDAR projects behind objects

**Fix**:
- Check sign of translation values (x positive = forward)
- Verify frame conventions match (ROS REP-103)

### Large rotation error

**Symptom**: Projections spiral/rotate incorrectly

**Fix**:
- Check rotation order (roll, pitch, yaw)
- Verify angle units (radians not degrees)
- Re-measure angles carefully

---

## Best Practices

1. ✅ **Start simple**: Use rough manual measurement first
2. ✅ **Iterate**: Make small adjustments, test, repeat
3. ✅ **Document**: Record calibration values and date
4. ✅ **Version control**: Commit calibration changes to git
5. ✅ **Test thoroughly**: Validate at multiple distances and angles
6. ✅ **Recalibrate**: After hardware changes, impacts, or drift

---

## Need Help?

If calibration is still poor after following this guide:

1. Check hardware:
   - Sensors securely mounted (no vibration)
   - Consistent orientation
   - No physical obstructions

2. Verify sensor data:
   - LiDAR publishing points
   - Camera publishing images
   - Timestamps synchronized

3. Debug projection:
   - Simple test: Project known 3D point
   - Verify projection function
   - Check camera intrinsics

4. Ask for help:
   - Include debug image screenshot
   - Share calibration YAML values
   - Provide TF tree diagram
