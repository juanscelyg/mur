cmake_minimum_required(VERSION 2.8.3)
project(mur_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  geometry_msgs
  tf
  std_msgs
  nav_msgs
  visualization_msgs)

add_message_files(
  FILES
  FloatStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_install_python(PROGRAMS  scripts/mur_altitude_controller.py
                                scripts/mur_control_mixer.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
