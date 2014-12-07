cmake_minimum_required(VERSION 2.8.3)
project(youbot_rw_rqt_gui)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
catkin_python_setup()

add_message_files(
  FILES
  rw_node.msg
  rw_node_state.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)





install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/youbot_rw_rqt_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


