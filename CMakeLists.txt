cmake_minimum_required(VERSION 2.8.3)
project(myface_recognition)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/face_search.py
  scripts/face_register.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY
    ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
