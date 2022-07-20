cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter_tutorial_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs#   visualization_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/kalman_filter_parameter.cfg
  config/dynamic_reconfigure/observation_parameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kalman_filter_tutorial_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_executable(kalman_filter_tutorial_ros_virtual_environment src/virtual_environment/virtual_environment.cpp)
target_link_libraries(kalman_filter_tutorial_ros_virtual_environment  ${catkin_LIBRARIES})
add_dependencies(kalman_filter_tutorial_ros_virtual_environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(kalman_filter_tutorial_ros_tracker src/tracker.cpp
                                              src/kalman_filter.cpp)
target_link_libraries(kalman_filter_tutorial_ros_tracker  ${catkin_LIBRARIES})
add_dependencies(kalman_filter_tutorial_ros_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})