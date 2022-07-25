cmake_minimum_required(VERSION 3.1.3)
project(mycobot_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
