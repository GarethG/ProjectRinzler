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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} src/lib/libRBIO.a)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
INCLUDE_DIRECTORIES(src/ $ENV{HOME}/ProjectRinzler/Roboard/)
#LINK_DIRECTORIES(~/ProjectRinzler/Roboard/Lib/)
rosbuild_add_executable(motor src/motor.cpp $ENV{HOME}/ProjectRinzler/Roboard/common.cpp $ENV{HOME}/ProjectRinzler/Roboard/pwm.cpp $ENV{HOME}/ProjectRinzler/Roboard/io.cpp $ENV{HOME}/ProjectRinzler/Roboard/pwmdx.cpp)
#target_link_libraries(motor common RBIO)#${PROJECT_NAME})



