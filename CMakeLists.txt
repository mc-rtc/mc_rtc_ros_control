#
# Copyright 2021 mc_rtc development team
#

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)

project(mc_rtc_ros_control LANGUAGES CXX VERSION 0.1.0)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

find_package(mc_rtc REQUIRED)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} mc_rtc::mc_control)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
