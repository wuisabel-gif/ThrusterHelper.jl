cmake_minimum_required(VERSION 3.8)
project(thruster_allocator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(allocator_node src/allocator_node.cpp)
target_include_directories(allocator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(allocator_node rclcpp geometry_msgs std_msgs)

install(TARGETS allocator_node DESTINATION lib/${PROJECT_NAME})

ament_package()
