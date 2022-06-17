# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)

#---------------------------------------------------------------------------------------
# Start livox_ros_driver project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver version: ${LIVOX_ROS_DRIVER_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    livox_sdk
    message_generation
    nodelet
    pluginlib
    rosbag
    roscpp
    rospy
    sensor_msgs
    std_msgs
    # The version of PCL included by pcl_ros conflicts with the version used below.
    # These libraries pull in the system version of the PCL headers, causing build issues.
    # pcl_ros
)

## Find pcl lib.
find_package(PCL 1.12 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
    livox_sdk
    message_runtime
    nodelet
    pluginlib
    roscpp
    rospy
    std_msgs
    # The version of PCL included by pcl_ros conflicts with the version used below.
    # These libraries pull in the system version of the PCL headers, causing build issues.
    # pcl_ros
)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(
    ${PROJECT_NAME}_node
    ""
)

#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
add_dependencies(
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_generate_messages_cpp
)

#---------------------------------------------------------------------------------------
# source file
#---------------------------------------------------------------------------------------
target_sources(${PROJECT_NAME}_node
    PRIVATE
    livox_ros_driver/lvx_file.cpp
    livox_ros_driver/ldq.cpp
    livox_ros_driver/lds.cpp
    livox_ros_driver/lds_lvx.cpp
    livox_ros_driver/lds_lidar.cpp
    livox_ros_driver/lds_hub.cpp
    livox_ros_driver/lddc.cpp
    livox_ros_driver/livox_ros_driver.cpp
    timesync/timesync.cpp
    timesync/user_uart/user_uart.cpp
    common/comm/comm_protocol.cpp
    common/comm/sdk_protocol.cpp
    common/comm/gps_protocol.cpp
)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    common
    common/rapidjson
    comon/rapdidxml
    common/comm
    timesync
    timesync/user_uart
    livox_ros_driver
)

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
    ${Boost_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
)

#---------------------------------------------------------------------------------------
# ADD NODELET LIBRARIES
#---------------------------------------------------------------------------------------
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    common
    common/rapidjson
    comon/rapdidxml
    common/comm
    timesync
    timesync/user_uart
    livox_ros_driver
    )

add_library(${PROJECT_NAME}_nodelet
    livox_ros_driver/livox_ros_driver_nodelet.cpp
    livox_ros_driver/lvx_file.cpp
    livox_ros_driver/ldq.cpp
    livox_ros_driver/lds.cpp
    livox_ros_driver/lds_lvx.cpp
    livox_ros_driver/lds_lidar.cpp
    livox_ros_driver/lds_hub.cpp
    livox_ros_driver/lddc.cpp
    livox_ros_driver/livox_ros_driver.cpp
    timesync/timesync.cpp
    timesync/user_uart/user_uart.cpp
    common/comm/comm_protocol.cpp
    common/comm/sdk_protocol.cpp
    common/comm/gps_protocol.cpp
    )

target_link_libraries(${PROJECT_NAME}_nodelet
    ${Boost_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_nodelet_EXPORTED_TARGETS})

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    launch
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES livox_ros_driver_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#---------------------------------------------------------------------------------------
# Testing
#---------------------------------------------------------------------------------------

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
