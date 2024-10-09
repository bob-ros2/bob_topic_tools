cmake_minimum_required(VERSION 3.8)
project(bob_topic_tools)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  ${PROJECT_NAME}/filter
  ${PROJECT_NAME}/terminal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
