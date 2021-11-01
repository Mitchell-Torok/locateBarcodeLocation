cmake_minimum_required(VERSION 3.5)
project(locateBarcodeLocation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zbar_ros_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(qr_custom_message REQUIRED)


add_executable(locateBarcodeLocation src/locate_barcode.cpp)
ament_target_dependencies(locateBarcodeLocation rclcpp std_msgs geometry_msgs sensor_msgs zbar_ros_interfaces cv_bridge qr_custom_message)

install(TARGETS
  locateBarcodeLocation
  DESTINATION lib/${PROJECT_NAME})
ament_package()
