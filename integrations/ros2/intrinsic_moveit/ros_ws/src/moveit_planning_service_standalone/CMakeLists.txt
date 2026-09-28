cmake_minimum_required(VERSION 3.8)
project(moveit_planning_service)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(INTRINSIC_MOVEIT_DIR "/opt/intrinsic-moveit" CACHE PATH "Checkout of intrinsic-ai/intrinsic-moveit")
set(UPSTREAM_DIR "${INTRINSIC_MOVEIT_DIR}/moveit_planning_service")
if(NOT EXISTS "${UPSTREAM_DIR}/src/grasp_planning_pipeline.cpp")
  message(FATAL_ERROR "INTRINSIC_MOVEIT_DIR=${INTRINSIC_MOVEIT_DIR} does not contain moveit_planning_service sources")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_planning_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(moveit_planning_node
  src/moveit_planning_node_standalone.cpp
  ${UPSTREAM_DIR}/src/grasp_planning_pipeline.cpp
  ${UPSTREAM_DIR}/src/generate_box_grasp_poses.cpp
  ${UPSTREAM_DIR}/src/object_geometry.cpp
)
target_include_directories(moveit_planning_node PRIVATE ${UPSTREAM_DIR}/include)
target_link_libraries(moveit_planning_node
  rclcpp::rclcpp
  Eigen3::Eigen
  ${moveit_msgs_TARGETS}
  ${moveit_core_TARGETS}
  ${moveit_ros_planning_TARGETS}
  ${moveit_task_constructor_core_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

install(TARGETS moveit_planning_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${UPSTREAM_DIR}/launch ${UPSTREAM_DIR}/rviz DESTINATION share/${PROJECT_NAME})

ament_package()
