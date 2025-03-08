cmake_minimum_required(VERSION 3.8)
project(bob_topic_tools)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg 
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
