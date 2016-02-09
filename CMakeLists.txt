cmake_minimum_required(VERSION 2.8.3)
project(asterx1_node)


## Activate c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        sensor_msgs
        visualization_msgs
        tf
        nav_msgs
)

#TODO togliere le cose che c'entrano con i marker, tf e odometry

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(iriutils REQUIRED)
find_package(asterx1_gps REQUIRED)
find_package(Ceres)

#TODO fare in modo che gpstk sia required
FIND_LIBRARY(gpstk_LIBRARY REQUIRED
    NAMES gpstk
    PATHS /usr/lib /usr/local/lib
)



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
add_message_files(
  FILES
  SatPr.msg
  SatPrArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS message_runtime
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asterx1_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###############
## Srcs list ##
###############

##
SET( SRCS_RAW_REC
	src/raw_receiver_node.cpp
	src/raw_receiver_main.cpp
)

##
SET( SRCS_VIZ_HELP
    src/viz_helper_node.cpp
    src/viz_helper_main.cpp
)

##SRCS files. Trilateration library
SET(SRCS_TR_LIB
    include/trilateration/src/trilateration.cpp
)

##
SET(SRCS_TRILAT_NODE
    src/trilateration_node.cpp
    src/trilateration_main.cpp
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

## cpp executable
add_executable(raw_receiver_node ${SRCS_RAW_REC} )
target_link_libraries(raw_receiver_node ${catkin_LIBRARIES} gpstk)
add_dependencies(raw_receiver_node iri_asterx1_gps_generate_messages_cpp)

add_executable(viz_helper ${SRCS_VIZ_HELP} )
target_link_libraries(viz_helper ${catkin_LIBRARIES})


IF(Ceres_FOUND)
    add_executable(trilat_node ${SRCS_TRILAT_NODE} ${SRCS_TR_LIB})
    target_link_libraries(trilat_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} gpstk)
	add_dependencies(raw_receiver_node asterx1_node_generate_messages_cpp)
ENDIF(Ceres_FOUND)
