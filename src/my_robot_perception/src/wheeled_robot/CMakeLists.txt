cmake_minimum_required(VERSION 3.8)
project(wheeled_robot)

# ------------------------------------------------------------------ #
#  C++ compiler flags
# ------------------------------------------------------------------ #
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------ #
#  ROS 2 dependencies
# ------------------------------------------------------------------ #
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)                     # Python node
find_package(geometry_msgs REQUIRED)             # ← FIXED typo
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ------------------------------------------------------------------ #
#  Install directories (launch, urdf, config, meshes, worlds)
# ------------------------------------------------------------------ #
install(
  DIRECTORY
    launch
    urdf
    config
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

# ------------------------------------------------------------------ #
#  Python node: diff_drive_publisher
# ------------------------------------------------------------------ #
install(
  PROGRAMS
    launch/robot_launch.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------ #
#  (Optional) If you ever add a C++ node, use this pattern:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp ...)
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
# ------------------------------------------------------------------ #

# ------------------------------------------------------------------ #
#  Testing (linting)
# ------------------------------------------------------------------ #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()