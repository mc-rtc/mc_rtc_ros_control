#
# Copyright 2021 mc_rtc development team
#

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)

project(mc_rtc_ros_control LANGUAGES CXX VERSION 0.1.0)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  INCLUDE_DIRS include
  LIBRARIES mc_rtc_ros_control
)

find_package(mc_rtc REQUIRED)

add_library(${PROJECT_NAME} src/ROSControlInterface.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_exe src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_exe PUBLIC mc_rtc::mc_control ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_exe ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
