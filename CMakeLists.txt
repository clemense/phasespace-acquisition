cmake_minimum_required(VERSION 2.8.3)
project(phasespace_acquisition)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    PhaseSpaceMarker.msg
    PhaseSpaceMarkerArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(libs)

set(LIB_PHASESPACE phasespace_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIB_PHASESPACE}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

set(BIN_TALKER phasespace_talker)
set(BIN_LISTENER phasespace_listener)

# Talker
add_executable(${BIN_TALKER}
  src/phasespace_talker.cpp
  src/phasespace_core.cpp
)
target_link_libraries(${BIN_TALKER} 
  ${catkin_LIBRARIES}
  owlsock
)
add_dependencies(${BIN_TALKER} 
  ${catkin_EXPORTED_TARGETS}
)

# Listener
add_executable(${BIN_LISTENER}
  src/phasespace_listener.cpp
  src/phasespace_core.cpp
)
target_link_libraries(${BIN_LISTENER}
  ${catkin_LIBRARIES}
  owlsock
)
add_dependencies(${BIN_LISTENER} 
  ${catkin_EXPORTED_TARGETS}
)

# Exported library
add_library(${LIB_PHASESPACE}
  src/phasespace_core.cpp
)
target_link_libraries(${LIB_PHASESPACE}
  ${catkin_LIBRARIES}
  owlsock
)
add_dependencies(${LIB_PHASESPACE} 
  ${catkin_EXPORTED_TARGETS}
)
