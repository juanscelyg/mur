cmake_minimum_required(VERSION 2.8.3)
project(mur_description)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roslib
    geometry_msgs
    message_generation
    tf
    std_msgs
    nav_msgs
    visualization_msgs
    dynamic_reconfigure)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_install_python(PROGRAMS  scripts/mur_sim_to_into.py
                                scripts/mur_sim_to_out.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY mesh robots urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
