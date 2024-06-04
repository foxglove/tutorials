cmake_minimum_required(VERSION 3.8)
project(diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(diagnostics_publisher src/main.cpp)
ament_target_dependencies(diagnostics_publisher rclcpp diagnostic_msgs std_msgs)

install(TARGETS diagnostics_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
