cmake_minimum_required(VERSION 3.0.2)
project(metacontrol_knowrob)

find_package(catkin REQUIRED COMPONENTS
  rosprolog
  rospy
  knowrob
)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/metacontrol.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
