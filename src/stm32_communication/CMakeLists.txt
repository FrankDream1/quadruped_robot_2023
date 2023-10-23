cmake_minimum_required(VERSION 3.0.2)
project(stm32_communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stm32_communication
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(STM32_Node src/stm32_communication.cpp src/UnitreeDriver.cpp)
target_link_libraries(STM32_Node
  ${catkin_LIBRARIES}
)