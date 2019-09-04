cmake_minimum_required(VERSION 2.8.3)
project(launcher_state_machine)

find_package(catkin REQUIRED COMPONENTS
             actionlib
             geometry_msgs
             mbf_msgs
             scrubber_msgs
             roscpp
             std_srvs
             )

catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES launcher_state_machine
		CATKIN_DEPENDS geometry_msgs launcher_msgs mbf_msgs db_msgs roscpp
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

## Declare a C++ library
add_library(StateMachine
            src/StateMachine.cpp
            )

add_library(ScrubberStates
            src/ScrubberStates.cpp
            )

add_library(Automatic
            src/Automatic.cpp)

add_library(Mapping
            src/Mapping.cpp)

target_link_libraries(Automatic ScrubberStates)
target_link_libraries(Mapping ScrubberStates)

target_link_libraries(StateMachine
                      ScrubberStates
                      Automatic
                      Mapping
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )
add_dependencies(StateMachine
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context

add_executable(launcher_state_machine_node
               src/launcherStateMachineNode.cpp)

add_dependencies(launcher_state_machine_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(launcher_state_machine_node
                      StateMachine)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
		TARGETS
		StateMachine
		launcher_state_machine_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

