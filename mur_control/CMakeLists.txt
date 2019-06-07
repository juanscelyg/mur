cmake_minimum_required(VERSION 2.8.3)
project(mur_control)

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

add_message_files(
  FILES
  FloatStamped.msg
  BoolStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MurControlMixer.cfg
  cfg/MurAltitudeControl.cfg
  cfg/MurPositionControl.cfg
  cfg/MurAttitudeControl.cfg
)

catkin_install_python(PROGRAMS  scripts/mur_altitude_control.py
                                scripts/mur_attitude_control.py
                                scripts/mur_position_control.py
                                scripts/mur_control_mixer.py
                                scripts/mur_ekf.py
                                scripts/mur_aruco_detector.py
                                scripts/mur_ident_lineales.py
                                scripts/mur_barometer_parse.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
