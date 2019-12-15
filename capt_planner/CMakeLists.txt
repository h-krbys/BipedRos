cmake_minimum_required(VERSION 3.0)
project(capt_planner)

set(RVIZ_SOURCES ../choreonoid_rvizplugin/src/RvizPublisher.cpp)

set(CAPT_DIR "" CACHE PATH "root directory of Capturability")
if(EXISTS ${CAPT_DIR})
  foreach(directory ${CAPT_DIR})
    file(GLOB_RECURSE files ${CAPT_DIR}/src/*.cpp ${CAPT_DIR}/plot/src/*.cpp)
    list(APPEND CAPT_SOURCES ${files})
  endforeach()
  include_directories(${CAPT_DIR}/include)
  include_directories(${CAPT_DIR}/plot/include)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
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
  ../choreonoid_rvizplugin/include
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
  add_executable(${target} ${ARGV} ${RVIZ_SOURCES})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target} ${catkin_LIBRARIES})
  install(TARGETS ${target}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

add_controller(GlobalPlanner src/GlobalPlanner.cpp)
add_controller(LocalPlanner  src/LocalPlanner.cpp)