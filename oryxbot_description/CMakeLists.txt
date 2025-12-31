cmake_minimum_required(VERSION 2.8.3)

project(oryxbot_description)

find_package(catkin REQUIRED
	geometry_msgs
	roscpp
	std_msgs 
	std_srvs
	message_generation
	arm_controller
	tf2_ros
	tf
)
## Generate services in the 'srv' folder
add_service_files(
	FILES
	FK.srv
	IK.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swiftpro_control
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs std_srvs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(fk_swiftpro src/fk_swiftpro.cpp)
add_executable(ik_swiftpro src/ik_swiftpro.cpp)
add_executable(home_pose src/home_pose.cpp)
add_executable(simple_picker src/simple_picker.cpp)
add_executable(pose_swiftpro src/pose_swiftpro.cpp)
add_executable(pick_ar_gazebo src/pick_ar_gazebo.cpp)


## --- 修改开始 ---

# 1. 解决 FK.h, IK.h 找不到的问题
# 这些节点依赖本包 (oryxbot_description) 生成的消息头文件
# 注意：${PROJECT_NAME} 就是 oryxbot_description
add_dependencies(fk_swiftpro ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ik_swiftpro ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(home_pose ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(simple_picker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pose_swiftpro ${PROJECT_NAME}_generate_messages_cpp)

# 2. 解决 arm_controller/move.h 找不到的问题
# 这些节点依赖 arm_controller 包生成的消息头文件
# 注意：前提是你的 find_package 里已经包含了 arm_controller
add_dependencies(pick_ar_gazebo arm_controller_generate_messages_cpp)
add_dependencies(pose_swiftpro arm_controller_generate_messages_cpp)

## --- 修改结束 ---
#  add_dependencies(fk_swiftpro swiftpro_control_generate_messages_cpp)
# add_dependencies(fk_swiftpro std_msgs_generate_messages_cpp)
# add_dependencies(ik_swiftpro swiftpro_control_generate_messages_cpp)
# add_dependencies(home_pose swiftpro_control_generate_messages_cpp)
# add_dependencies(simple_picker swiftpro_control_generate_messages_cpp)
# add_dependencies(pose_swiftpro swiftpro_control_generate_messages_cpp)
# add_dependencies(pick_ar_gazebo swiftpro_control_generate_messages_cpp)
 
target_link_libraries(fk_swiftpro ${catkin_LIBRARIES})
target_link_libraries(ik_swiftpro ${catkin_LIBRARIES})
target_link_libraries(home_pose ${catkin_LIBRARIES}) 
target_link_libraries(simple_picker ${catkin_LIBRARIES}) 
target_link_libraries(pose_swiftpro ${catkin_LIBRARIES}) 
target_link_libraries(pick_ar_gazebo ${catkin_LIBRARIES}) 

## Add cmake target dependencies of the executable
## same as for the library above

#foreach(dir config launch meshes urdf)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)
