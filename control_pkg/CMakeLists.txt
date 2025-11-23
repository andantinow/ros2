cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(el_interfaces REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(nmpc_engine_node src/nmpc_engine_node.cpp)
ament_target_dependencies(nmpc_engine_node
  rclcpp
  el_interfaces
  ackermann_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS nmpc_engine_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
