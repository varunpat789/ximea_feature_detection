cmake_minimum_required(VERSION 3.8)
project(ximea_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ximea_feature_detection src/ximea_feature_detection.cpp)
ament_target_dependencies(ximea_feature_detection rclcpp std_msgs sensor_msgs)

install(TARGETS
  ximea_feature_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
