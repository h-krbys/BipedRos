cmake_minimum_required(VERSION 2.8.12)
project(choreonoid_rvizplugin)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  choreonoid
  #choreonoid_rosplugin
  biped_msgs
  tf
)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++11)
endif()

catkin_package( CATKIN_DEPENDS
  roscpp
)

add_definitions(${CHOREONOID_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
)

# build plugin
add_cnoid_plugin(CnoidRvizPlugin SHARED
  src/RvizPlugin.cpp
  # src/RvizPublisher.cpp
)

target_link_libraries(CnoidRvizPlugin
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
  )

install(TARGETS CnoidRvizPlugin
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
)
# end build plugin

function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

# add_executable(RvizPublisher src/RvizPublisher.cpp)
# add_dependencies(RvizPublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(RvizPublisher ${catkin_LIBRARIES})
# install(TARGETS RvizPublisher
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

add_subdirectory(sample)
add_subdirectory(project)