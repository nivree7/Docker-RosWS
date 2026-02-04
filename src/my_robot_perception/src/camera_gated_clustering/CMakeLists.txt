cmake_minimum_required(VERSION 3.8)
project(camera_gated_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fusion_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io search segmentation)

file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/camera_gated_clustering/camera_gated_clustering_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/camera_gated_clustering/lidar_camera_fusion_node.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  camera_gated_clustering/camera_gated_clustering_node.py
  camera_gated_clustering/lidar_camera_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pcl_euclidean_clustering_node
  src/pcl_euclidean_clustering_node.cpp
)
ament_target_dependencies(pcl_euclidean_clustering_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  fusion_msgs
  pcl_conversions
)
target_include_directories(pcl_euclidean_clustering_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_euclidean_clustering_node ${PCL_LIBRARIES})

install(TARGETS
  pcl_euclidean_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_projection
    test/test_projection.py
  )
endif()

ament_package()
