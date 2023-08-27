cmake_minimum_required(VERSION 3.0.2)
project(regulated_pure_pursuit_controller)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  tf2
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/RegulatedPurePursuitController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES regulated_pure_pursuit_controller
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    rospy
    tf2
    tf2_ros
)

add_library(regulated_pure_pursuit_controller
  src/regulated_pure_pursuit_controller.cpp
)
add_dependencies(regulated_pure_pursuit_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(regulated_pure_pursuit_controller
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(regulated_pure_pursuit_controller
  ${catkin_LIBRARIES}
)

install(TARGETS regulated_pure_pursuit_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES regulated_pure_pursuit_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)