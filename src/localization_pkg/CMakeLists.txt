cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(el_interfaces REQUIRED)

include_directories(include)

# Build Estimator Node
add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  el_interfaces
)

# Install
install(TARGETS estimator_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
