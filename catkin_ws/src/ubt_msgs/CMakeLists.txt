cmake_minimum_required(VERSION 2.8.3)
project(ubt_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
        FILES
        action_report.msg
        sensor_report.msg
        gyro_report.msg
        tts_data.msg
        asr_iat_data.msg
	angles_set.msg
        motion_planning_speed.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        action.srv
        battery_read.srv
        board_read.srv
        combine_request.srv
        env_read.srv
        ext_read.srv
        gyro_read.srv
        sensor_calibrate_set.srv
        sensor_report_enable.srv
        led.srv
        led_reply.srv
        i2c_upgrade.srv
        hts_status_change.srv
        ubt_interrupt_action.srv
        vision_take_photo.srv
        vision_detect.srv
        task_configuration_set.srv
        task_configuration_get.srv
        servo_read.srv
        servo_write.srv
        servo_active.srv
        servo_offset_write.srv
        servo_offset_read.srv
        version_read.srv
        vision_ai_event.srv
        vision_recognition.srv
        vision_monitor.srv
        vision_status.srv
	    vision_get.srv
	    vision_set.srv
		vision_stream.srv
		vision_report_int.srv
		vision_report_str.srv
		vision_report_expression.srv
        vision_report_color.srv
        voice_asr_iat_result.srv
        voice_asr_start.srv
        voice_asr_stop.srv
        voice_asr_subscribe.srv
		voice_asr_offlinesyntax_set.srv
		voice_asr_offlinesyntax_get.srv
		voice_asr_offlinesyntax_grammar_get.srv
        voice_tts_status.srv
        voice_tts_start.srv
        voice_tts_stop.srv
        voice_iat_start.srv
        voice_iat_stop.srv
        voice_iat_detectedmsg.srv
        volume_get.srv
        volume_set.srv
        value_set.srv
        value_get.srv
        enable_set.srv
        enable_get.srv
        device_addr.srv
        move_status.srv
        music_play.srv
        music_status.srv
        motion_planning.srv
        motion_planning_status.srv
	string_set.srv
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
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ubt_msgs
        CATKIN_DEPENDS message_runtime
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
#${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ubt_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(ubt_msgs_server src/hal_servo_server.cpp src/utils.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(ubt_msgs_server hal_servo_gencpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(ubt_msgs_server
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ubt_msgs_server
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/ubt_msgs/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  doc/TYPE_MAP
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ubt_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
