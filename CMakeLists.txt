cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(phasespace_talker
  src/phasespace_talker.cpp
  src/phasespace_core.cpp
)

target_link_libraries(phasespace_talker 
  ${PROJECT_SOURCE_DIR}/libs/libowlsock.so
)

rosbuild_add_compile_flags(phasespace_talker "-std=c++0x")

rosbuild_add_executable(phasespace_listener
  src/phasespace_listener.cpp
  src/phasespace_core.cpp
)

target_link_libraries(phasespace_listener 
  ${PROJECT_SOURCE_DIR}/libs/libowlsock.so
)

rosbuild_add_compile_flags(phasespace_listener "-std=c++0x")

rosbuild_add_executable(phasespace_rviz_publisher
  src/phasespace_rviz_publisher.cpp
)

target_link_libraries(phasespace_rviz_publisher 
  ${PROJECT_SOURCE_DIR}/libs/libowlsock.so
)

rosbuild_add_compile_flags(phasespace_rviz_publisher "-std=c++0x")

