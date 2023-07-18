cmake_minimum_required(VERSION 3.0.2)
project(inverted_pendulum_controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_generation rospy  message_runtime)
install(FILES)
