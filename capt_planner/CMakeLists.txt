cmake_minimum_required(VERSION 3.0)
project(capt_planner)

set(CAPT_DIR "" CACHE PATH "root directory of Capturability")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  choreonoid
  choreonoid_rvizplugin
  footstep_planner
)
find_package(EXPAT REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
   roscpp
   choreonoid_rvizplugin
  DEPENDS
   OpenCV
)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++11)
endif()

add_definitions(${CHOREONOID_DEFINITIONS})

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EXPAT_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
)

function(add_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_executable(${target} ${ARGV})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target} ${catkin_LIBRARIES})
  install(TARGETS ${target}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

add_controller(GlobalPlanner src/GlobalPlanner.cpp)