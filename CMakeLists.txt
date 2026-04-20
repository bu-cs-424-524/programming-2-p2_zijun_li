cmake_minimum_required(VERSION 3.0.2)
project(p2_lizijun)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib_msgs
  tf
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  script/p2a_nav_goals.py
  script/p2b_ball_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config misc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
