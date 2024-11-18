cmake_minimum_required(VERSION 2.8.3)

project(urdf7)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
)

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
)

catkin_install_python(PROGRAMS scripts/robot_joint_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
