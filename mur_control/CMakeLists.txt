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
)

catkin_install_python(PROGRAMS  scripts/mur_attitude_control.py
                                scripts/mur_attitude_tele.py
                                scripts/mur_supervisor.py
                                scripts/mur_control_mixer.py
                                scripts/mur_ekf.py
                                scripts/mur_aruco_detector.py
                                scripts/mur_ident_lineales.py
                                scripts/mur_barometer_parse.py
                                scripts/mur_pose2d_parse.py
                                scripts/mur_pid_acc.py
                                scripts/mur_sonar_converter.py
                                scripts/mur_imu_parse.py
                                scripts/mur_gamepad_parse.py
                                scripts/mur_gamepad_force.py
                                scripts/mur_stop_motors.py
                                scripts/mur_test_paths.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
