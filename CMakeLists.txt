cmake_minimum_required(VERSION 2.8.3)

project(easy_drive)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_python_setup()

checkPackage("dynamic_reconfigure" "")

# Add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/drive_teleop.cfg
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs actionlib_msgs std_msgs dynamic_reconfigure
)

catkin_install_python(
  PROGRAMS nodes/drive_teleop nodes/cmd_vel_mux nodes/config_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install firmware files
install(DIRECTORY drive_firmware/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/drive_firmware
)