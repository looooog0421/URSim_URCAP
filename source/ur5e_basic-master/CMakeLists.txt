cmake_minimum_required(VERSION 2.8.0)
project(ur5e_basic)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	actionlib
	control_msgs
	sensor_msgs
	trac_ik_lib
	geometry_msgs
	trajectory_msgs
	rosbag
	std_msgs
	tf
	rviz
	robotiq_85_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES learning_topic
	#  DEPENDS system_lib
)

add_executable(ur_test src/ur_test.cpp src/UR5e.cpp)
add_dependencies(ur_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_test ${catkin_LIBRARIES})

add_executable(ur5e_server_node src/ur5e_server.cpp src/UR5e.cpp)
add_dependencies(ur5e_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur5e_server_node ${catkin_LIBRARIES})

add_executable(erase_blackboard_node src/erase_blackboard.cpp src/UR5e.cpp)
add_dependencies(erase_blackboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(erase_blackboard_node ${catkin_LIBRARIES})

add_executable(erase_blackboard_constant_contact_node src/erase_blackboard_constant_contact.cpp src/UR5e.cpp)
add_dependencies(erase_blackboard_constant_contact_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(erase_blackboard_constant_contact_node ${catkin_LIBRARIES})