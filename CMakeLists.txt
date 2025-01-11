#
# Copyright 2021 mc_rtc development team
#

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(mc_rtc_ros_control LANGUAGES CXX VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mc_rtc REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ROSControlInterface.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC ${ament_INCLUDE_DIRS} $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${ament_LIBRARIES})

add_executable(${PROJECT_NAME}_exe src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_exe PUBLIC mc_rtc::mc_control ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
