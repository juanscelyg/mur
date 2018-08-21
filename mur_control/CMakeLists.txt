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

catkin_package()

catkin_install_python(PROGRAMS  scripts/mur_altitude_controller.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
