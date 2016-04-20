cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)

project(imu_talk)

## Find catkin and any catkin packages
find_package(catkin REQUIRED cmake_modules COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)
find_package(Eigen REQUIRED)
## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(
	DEPENDS Eigen
)

## Build talker and listener
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

add_executable(listener src/listener.cpp)
target_link_libraries(listener
	${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(LSM6 src/LSM6DS33.cpp)
add_library(LIS3 src/LIS3MDL.cpp)
add_library(I2C src/I2CBus.cpp)
add_library(IMU src/MinIMU9.cpp)
add_library(IMU_OP src/minimu9-ahrs.cpp)
add_executable(talker src/talker.cpp)
target_link_libraries(talker
	${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
	I2C
	LSM6
	LIS3
	IMU
	IMU_OP
)

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#target_link_libraries(tf_broadcaster
#	${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#	I2C
#	LSM6
#	LIS3
#	IMU
#	IMU_OP
#)
