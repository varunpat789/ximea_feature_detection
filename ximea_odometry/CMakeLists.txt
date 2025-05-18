cmake_minimum_required(VERSION 3.8)
project(ximea_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create LBP library
add_library(lbp_lib src/lbp.cpp)
target_include_directories(lbp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lbp_lib ${OpenCV_LIBS})

# Create ROS2 node executable
add_executable(ximea_optical_flow src/ximea_optical_flow.cpp)
ament_target_dependencies(ximea_optical_flow 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV
)

# Create standalone LBP test executable (non-ROS)
add_executable(lbp_test src/lbp_test.cpp)
target_link_libraries(lbp_test lbp_lib ${OpenCV_LIBS})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  ximea_optical_flow
  lbp_lib
  lbp_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()