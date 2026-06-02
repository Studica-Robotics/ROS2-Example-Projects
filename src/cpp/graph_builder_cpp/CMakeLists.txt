cmake_minimum_required(VERSION 3.8)
project(graph_builder_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Header-only JSON library — sudo apt install nlohmann-json3-dev
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(graph_builder src/graph_builder_node.cpp)
target_compile_features(graph_builder PUBLIC cxx_std_17)
ament_target_dependencies(graph_builder
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs)
target_link_libraries(graph_builder nlohmann_json::nlohmann_json)

install(TARGETS graph_builder DESTINATION lib/${PROJECT_NAME})

ament_package()
