cmake_minimum_required(VERSION 3.0)
project(capt_planner)

set(CAPT_DIR "" CACHE PATH "root directory of Capturability")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  choreonoid_rvizplugin
  footstep_planner
)
find_package(EXPAT REQUIRED)

catkin_package( CATKIN_DEPENDS
  roscpp
  choreonoid_rvizplugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXPAT_INCLUDE_DIRS}
)