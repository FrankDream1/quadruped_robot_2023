cmake_minimum_required(VERSION 3.0.2)
project(hardware_ctrl)

add_compile_options(-std=c++14)
set(yaml-cpp_DIR "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  yaml-cpp
  serial
  unitree_legged_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs 
  roscpp
  rospy
  unitree_legged_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(lib)

add_executable(motor_communication src/motor_communication.cpp)
add_executable(imu_communication src/imu_communication.cpp)
add_executable(dogstandup src/standup.cpp)
add_executable(STM32_Node src/stm32_communication.cpp src/UnitreeDriver.cpp)

add_dependencies(motor_communication 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(dogstandup 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dogstandup 
  ${catkin_LIBRARIES}
  libUnitreeMotorSDK_M80106_amd64.so
)
target_link_libraries(motor_communication 
  ${catkin_LIBRARIES}
  libUnitreeMotorSDK_M80106_amd64.so
)
target_link_libraries(imu_communication 
  ${catkin_LIBRARIES}
  libUnitreeMotorSDK_M80106_amd64.so
)
target_link_libraries(STM32_Node
  ${catkin_LIBRARIES}
)
# 链接静态库
target_link_libraries(motor_communication ${YAML_CPP_LIBRARIES})
target_link_libraries(dogstandup ${YAML_CPP_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
