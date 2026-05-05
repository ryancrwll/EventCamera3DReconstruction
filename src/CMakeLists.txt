cmake_minimum_required(VERSION 3.8)
project(event_3d_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)

# Find External Libraries
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Build the executables
add_executable(test_rec src/test_rec.cpp)
add_executable(triangulate src/triangulate.cpp)
add_executable(check_sync src/check_sync.cpp) # <--- NEW SYNC CHECKER

# Link dependencies
ament_target_dependencies(test_rec
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  OpenCV
  nlohmann_json
)

ament_target_dependencies(triangulate
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  OpenCV
  nlohmann_json
)

# <--- NEW DEPENDENCIES FOR SYNC CHECKER --->
ament_target_dependencies(check_sync
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  OpenCV
  nlohmann_json
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Install rules
install(TARGETS test_rec triangulate check_sync # <--- ADDED TO INSTALL
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()