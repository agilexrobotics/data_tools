cmake_minimum_required(VERSION 2.8.3)
project(data_tools)
add_definitions(-w)
set(USE_NOETIC true)
set(USE_LIFT false)

set(CMAKE_BUILD_TYPE "Release")
if(USE_NOETIC)
  set(CMAKE_CXX_FLAGS "-std=c++14")
else()
  set(CMAKE_CXX_FLAGS "-std=c++11")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

if(USE_NOETIC)
  ADD_DEFINITIONS(-D_USENOETIC=\"${USE_NOETIC}\")
endif()
if(USE_LIFT)
  ADD_DEFINITIONS(-D_USELIFT=\"${USE_LIFT}\")
endif()

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  rosbag
  data_msgs
  )
if(USE_LIFT)
  find_package(bt_task_msgs)
endif()
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
# find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(jsoncpp REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

set(lib_DIR /usr/lib/x84_64-linux-gnu)

link_directories(${lib_DIR})

# add_message_files(
#   DIRECTORY msg
#   FILES
#   .msg
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL EIGEN3 jsoncpp
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# set(CSF_DIR "/usr/local")
# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Jsoncpp_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${Jsoncpp_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}_dataCapture src/dataCapture.cpp)
add_dependencies(${PROJECT_NAME}_dataCapture ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataCapture ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libjsoncpp.so)

add_executable(${PROJECT_NAME}_dataSync src/dataSync.cpp)
add_dependencies(${PROJECT_NAME}_dataSync ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataSync ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_dataPublish src/dataPublish.cpp)
add_dependencies(${PROJECT_NAME}_dataPublish ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataPublish ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libjsoncpp.so)

#############
## install ##
#############
install(TARGETS ${PROJECT_NAME}_dataCapture ${PROJECT_NAME}_dataSync ${PROJECT_NAME}_dataPublish
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
