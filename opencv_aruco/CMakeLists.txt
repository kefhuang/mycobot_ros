cmake_minimum_required(VERSION 2.8.3)
project(opencv_aruco)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
  cv_bridge
)

add_message_files(FILES
  Pose.msg
)


generate_messages(DEPENDENCIES std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
catkin_install_python(PROGRAMS
  scripts/calibrate_camera.py
  scripts/generate_aruco_markers.py
  scripts/pose_estimation.py
  scripts/generate_print_aruco_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# find_package(OpenCV REQUIRED)
# add_executable(opencv_camera src/opencv_camera)
# target_link_libraries(opencv_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_executable(camera_display src/camera_display)
# target_link_libraries(camera_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
