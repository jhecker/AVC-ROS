cmake_minimum_required(VERSION 2.8.3)
project(abridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(
  abridge src/ABridge.cpp src/USBSerial.cpp
)

target_link_libraries(
  abridge
  ${catkin_LIBRARIES}
)

install(TARGETS abridge
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
