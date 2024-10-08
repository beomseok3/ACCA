cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(cropbox src/cropbox_bs.cpp)
ament_target_dependencies(cropbox rclcpp std_msgs sensor_msgs)


install(TARGETS
  cropbox
  DESTINATION lib/${PROJECT_NAME})
ament_package()
