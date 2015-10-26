cmake_minimum_required(VERSION 2.8.3)
project(pioneer_utils)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf )
catkin_package(
	
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_p3at src/teleop_p3at.cpp)
target_link_libraries(teleop_p3at ${catkin_LIBRARIES})

add_executable(moving_alone src/moving_alone.cpp)
target_link_libraries(moving_alone ${catkin_LIBRARIES})
