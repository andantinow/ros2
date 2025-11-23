cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(raceline_server_node src/raceline_server_node.cpp)
ament_target_dependencies(raceline_server_node
  rclcpp
  nav_msgs
  geometry_msgs
  ament_index_cpp
)

# Install Node
install(TARGETS raceline_server_node
  DESTINATION lib/${PROJECT_NAME})

# Install Data Folder (CSV 파일 복사)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
