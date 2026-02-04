#!/bin/bash
# Build script for camera-gated clustering packages

set -e  # Exit on error



# Navigate to workspace
cd ~/ros2_ws

# Build fusion_msgs first (dependency)
echo "Step 1/3: Building fusion_msgs..."
colcon build --packages-select fusion_msgs --symlink-install
echo "✅ fusion_msgs built successfully"
echo ""

# Source to make fusion_msgs available
source install/setup.bash

# Build camera_gated_clustering
echo "Step 2/3: Building camera_gated_clustering..."
colcon build --packages-select camera_gated_clustering --symlink-install
echo "✅ camera_gated_clustering built successfully"
echo ""

# Source again
source install/setup.bash

# Run tests (optional)
echo "Step 3/3: Running tests..."
colcon test --packages-select fusion_msgs camera_gated_clustering
colcon test-result --verbose
echo ""

echo "========================================="
echo "✅ Build Complete!"
echo "========================================="
echo ""
echo "To use the packages, run:"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
echo "To launch the fusion system:"
echo "  ros2 launch camera_gated_clustering sensor_fusion_full.launch.py"
echo ""
echo "========================================="
