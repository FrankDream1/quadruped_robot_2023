cmake_minimum_required(VERSION 3.0.2)
project(mpc_ctrl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  unitree_legged_msgs
)

find_package(Eigen3)
find_package(OsqpEigen REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/osqp
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs 
  roscpp
  rospy
  unitree_legged_msgs
)

add_library(dog_lib
  include/Kinematics.h
  src/Kinematics.cpp
  include/Param.h
  include/CtrlStates.h
  src/Utils.cpp
  include/Utils.h
  src/RobotControl.cpp
  include/RobotControl.h
  src/BasicEKF.cpp
  include/BasicEKF.h
  src/ConvexMpc.cpp
  include/ConvexMpc.h      
)

add_library(hardware_dog_lib
  include/HardwareROS.h
  src/HardwareROS.cpp
)            

add_executable(mpc_ctrl src/MainHardware.cpp)

target_link_libraries(dog_lib 
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

target_link_libraries(hardware_dog_lib 
  dog_lib
)

target_link_libraries(mpc_ctrl
  hardware_dog_lib
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)