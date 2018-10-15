cmake_minimum_required(VERSION 2.8.3)
project(realsense_subscriber)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

find_package( PCL REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  DEPENDS system_lib roscpp
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs
)


include_directories(
#  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
)

add_executable(realsense_subscriber_node src/main.cpp)
set_target_properties(realsense_subscriber_node PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(realsense_subscriber_node 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
