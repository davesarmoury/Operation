cmake_minimum_required(VERSION 3.0.2)
project(operation_control)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES operation_control
#  CATKIN_DEPENDS moveit_commander rospy tf twitch_chat_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/frame_transformer.py
  scripts/action_coordinator.py
  scripts/arm_controller.py
  scripts/gripper_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/controller.launch)
endif()
