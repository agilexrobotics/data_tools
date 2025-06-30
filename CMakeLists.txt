cmake_minimum_required(VERSION 3.5)
project(data_tools)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(USE_LIFT false)
if(USE_LIFT)
  ADD_DEFINITIONS(-D_USELIFT=\"${USE_LIFT}\")
endif()

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
# pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
if(NOT TARGET JsonCpp::JsonCpp)
    find_package(jsoncpp REQUIRED QUIET)
endif()
#find_package(jsoncpp REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(data_msgs REQUIRED)
if(USE_LIFT)
  find_package(bt_task_msgs REQUIRED)
endif()
#find_package(mcap REQUIRED)
#pkg_check_modules(LZ4 REQUIRED)
#find_library(LZ4_LIBRARY
#  NAMES lz4 liblz4
#)
#find_package(zstd REQUIRED)

#find_package(rosbag2_storage_mcap REQUIRED)
#find_package(mcap_vendor REQUIRED)
include_directories(
	include
)


add_executable(${PROJECT_NAME}_dataCapture src/dataCapture.cpp)
ament_target_dependencies(${PROJECT_NAME}_dataCapture rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL data_msgs cv_bridge)
target_link_libraries(${PROJECT_NAME}_dataCapture "${cpp_typesupport_target}" ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CURL_LIBRARIES} libjsoncpp.so)

add_executable(${PROJECT_NAME}_dataSync src/dataSync.cpp)
ament_target_dependencies(${PROJECT_NAME}_dataSync rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL data_msgs cv_bridge)
target_link_libraries(${PROJECT_NAME}_dataSync ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CURL_LIBRARIES} libjsoncpp.so)

add_executable(${PROJECT_NAME}_dataPublish src/dataPublish.cpp)
ament_target_dependencies(${PROJECT_NAME}_dataPublish rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL data_msgs cv_bridge)
target_link_libraries(${PROJECT_NAME}_dataPublish ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CURL_LIBRARIES} libjsoncpp.so)

#add_executable(${PROJECT_NAME}_readMcap src/readMcap.cpp)
#ament_target_dependencies(${PROJECT_NAME}_readMcap rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 #tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL cv_bridge)
#target_link_libraries(${PROJECT_NAME}_readMcap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libjsoncpp.so mcap
#${LZ4_LIBRARY}
#zstd)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_dataCapture ${PROJECT_NAME}_dataSync ${PROJECT_NAME}_dataPublish #${PROJECT_NAME}_readMcap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

file(GLOB python_scripts "scripts/*.py")
install(PROGRAMS
  ${python_scripts}
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

