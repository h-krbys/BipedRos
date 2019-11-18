cmake_minimum_required(VERSION 2.8.12)
project(biped_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  RvArrow.msg
  RvCircle.msg
  RvLine.msg
  RvPoint.msg
  RvSquare.msg
  RvStl.msg
)

generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package( CATKIN_DEPENDS
  roscpp
  visualization_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)