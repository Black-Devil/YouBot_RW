cmake_minimum_required(VERSION 2.8.3)
project(rqt_bag_test)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_bag_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
