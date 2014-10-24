cmake_minimum_required(VERSION 2.8.3)
project(serial_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  puppeteer_msgs)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_node src/serial_node.cpp)
add_executable(multi_serial_node src/multi_serial_node.cpp)
add_executable(multi_serial_topic src/multi_serial_topic.cpp)

add_dependencies(serial_node puppeteer_msgs_gencpp)
add_dependencies(multi_serial_node puppeteer_msgs_gencpp)
add_dependencies(multi_serial_topic puppeteer_msgs_gencpp)

target_link_libraries(serial_node ${catkin_LIBRARIES})
target_link_libraries(multi_serial_node ${catkin_LIBRARIES})
target_link_libraries(multi_serial_topic ${catkin_LIBRARIES}) 
