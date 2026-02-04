#!/usr/bin/env bash
echo "working"
ros2 run lidar_detection preprocessor_node \
  --ros-args \
  -r input_pointcloud:=/ouster/points
echo "Preprocessor node started."

ros2 run lidar_detection ground_removal_node \
  --ros-args \
  -r input_pointcloud:=/ouster/points
echo "Ground removal node started."

ros2 run lidar_detection cluster_extraction_node \
  --ros-args \
  -r input_pointcloud:=/ouster/points
echo "Cluster extraction node started."

ros2 run lidar_detection bounding_box_node \
  --ros-args \
  -r input_pointcloud:=/ouster/points
echo "Bounding box node started."