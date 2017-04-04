cmake_minimum_required(VERSION 2.8.3)
project(audio_file_player)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


# Generate actions in the 'action' folder
add_action_files(
  FILES
  AudioFilePlay.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/play_file_server.py
  scripts/volume_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
foreach (dir launch assets)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
