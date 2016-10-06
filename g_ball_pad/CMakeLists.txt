cmake_minimum_required(VERSION 2.8.3)
project(g_ball_pad)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg robotnik_msgs)

catkin_package(
CATKIN_DEPENDS
    roscpp
    diagnostic_updater
    robotnik_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(g_ball_pad src/g_ball_pad.cpp)
add_dependencies(g_ball_pad robotnik_msgs_generate_messages_cpp)
target_link_libraries(g_ball_pad ${catkin_LIBRARIES})
