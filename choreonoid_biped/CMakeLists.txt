cmake_minimum_required(VERSION 3.0)
project(choreonoid_biped)

set(CAPT_DIR "" CACHE PATH "root directory of Capturability")

set(RVIZ_SOURCES ../../../choreonoid_rvizplugin/src/RvizPublisher.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  choreonoid
  choreonoid_rvizplugin
)
find_package(EXPAT REQUIRED)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++11)
endif()

catkin_package( CATKIN_DEPENDS
  roscpp
  choreonoid_rvizplugin
)

add_definitions(${CHOREONOID_DEFINITIONS})

include_directories(
  include
  ../choreonoid_rvizplugin/include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${EXPAT_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
)
function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_compile_options(${target} PRIVATE -fPIC)
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES} ${EXPAT_LIBRARIES} CnoidRvizPlugin)
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

install(DIRECTORY project DESTINATION ${CHOREONOID_SHARE_SUBDIR})

if(EXISTS ${CAPT_DIR})
  foreach(directory ${CAPT_DIR})
    file(GLOB_RECURSE files ${CAPT_DIR}/src/*.cpp ${CAPT_DIR}/plot/src/*.cpp)
    list(APPEND CAPT_SOURCES ${files})
  endforeach()
  include_directories(${CAPT_DIR}/include)
  include_directories(${CAPT_DIR}/plot/include)
endif()

add_subdirectory(fall_avoidance)