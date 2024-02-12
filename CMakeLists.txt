cmake_minimum_required(VERSION 3.14)
project(trajectory_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/csv_loader.cpp
  include/trajectory_loader/csv_loader.hpp
  include/trajectory_loader/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/trajectory_loader_node.cpp
  include/trajectory_loader/trajectory_loader_node.hpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "trajectory_loader::TrajectoryLoaderNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
