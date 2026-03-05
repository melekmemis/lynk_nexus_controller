cmake_minimum_required(VERSION 3.0.2)
project(lynk_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lynk_nexus
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
