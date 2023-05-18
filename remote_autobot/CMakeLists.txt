cmake_minimum_required(VERSION 3.0.2)
project(remote_autobot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/remote_autobot_node.cpp)
target_link_libraries(remote_autobot_node ${catkin_LIBRARIES})

add_executable(web_server_autobot_node src/web_server_autobot_node.cpp)
target_link_libraries(web_server_autobot_node ${catkin_LIBRARIES})

add_executable(web_browser_autobot_node src/web_browser_autobot_node.cpp)
target_link_libraries(web_browser_autobot_node ${catkin_LIBRARIES})

install(TARGETS remote_autobot_node web_server_autobot_node # remote_move_autobot_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
