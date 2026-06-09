cmake_minimum_required(VERSION 3.8)
project(event_3d_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ==========================================
# 1. Find ROS 2 Packages (Ament)
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(image_geometry REQUIRED)

# ==========================================
# 2. Find External System Libraries
# ==========================================
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GSL REQUIRED) # <--- Added GSL requirement

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ==========================================
# 3. Build the Executables
# ==========================================
add_executable(test_rec src/test_rec.cpp)
add_executable(triangulate src/triangulate.cpp)
add_executable(check_sync src/check_sync.cpp)

# ==========================================
# 4. Link ROS 2 Dependencies (Ament ONLY)
# ==========================================
ament_target_dependencies(test_rec
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  image_geometry
  nlohmann_json
)

ament_target_dependencies(triangulate
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  image_geometry
  nlohmann_json
)

ament_target_dependencies(check_sync
  rclcpp
  rosbag2_cpp
  event_camera_msgs
  event_camera_codecs
  image_geometry
  nlohmann_json
)

# ==========================================
# 5. Link System Dependencies (OpenCV, GSL)
# ==========================================
target_link_libraries(test_rec
  ${OpenCV_LIBS}
  GSL::gsl
  GSL::gslcblas
)

target_link_libraries(triangulate
  ${OpenCV_LIBS}
  GSL::gsl
  GSL::gslcblas
)

target_link_libraries(check_sync
  ${OpenCV_LIBS}
)

# ==========================================
# 6. Install Rules
# ==========================================
install(TARGETS test_rec triangulate check_sync
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()