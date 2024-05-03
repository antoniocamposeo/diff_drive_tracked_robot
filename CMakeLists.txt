cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  catkin REQUIRED roslaunch
  std_msgs
  message_generation
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

roslaunch_add_file_check(launch)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS				
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)