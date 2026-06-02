cmake_minimum_required(VERSION 3.8)
project(diff_drive_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(diff_drive_node src/diff_drive_node.cpp)
target_compile_features(diff_drive_node PUBLIC cxx_std_17)
ament_target_dependencies(diff_drive_node rclcpp geometry_msgs std_msgs)

add_executable(odometry_node src/odometry_node.cpp)
target_compile_features(odometry_node PUBLIC cxx_std_17)
ament_target_dependencies(odometry_node rclcpp nav_msgs sensor_msgs std_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS
  diff_drive_node
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
