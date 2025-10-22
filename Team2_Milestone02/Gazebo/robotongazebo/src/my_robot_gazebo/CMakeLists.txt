cmake_minimum_required(VERSION 3.0.2)
project(my_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro
  robot_state_publisher
  joint_state_publisher_gui
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install launch files, urdf, meshes, and scripts
install(DIRECTORY launch urdf meshes models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/show_urdf_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
