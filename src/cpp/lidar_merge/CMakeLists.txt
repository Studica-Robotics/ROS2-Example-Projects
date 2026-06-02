cmake_minimum_required(VERSION 3.8)
project(lidar_merge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Lidar merge
add_executable(lidar_merge_node src/lidar_merge_node.cpp)
target_compile_features(lidar_merge_node PUBLIC cxx_std_17)
ament_target_dependencies(lidar_merge_node rclcpp sensor_msgs)

# Confidence scan gate
add_executable(confidence_scan_gate_node src/confidence_scan_gate_node.cpp)
target_compile_features(confidence_scan_gate_node PUBLIC cxx_std_17)
ament_target_dependencies(confidence_scan_gate_node rclcpp sensor_msgs nav_msgs tf2 tf2_ros)

install(TARGETS
  lidar_merge_node
  confidence_scan_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
