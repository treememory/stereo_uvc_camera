cmake_minimum_required(VERSION 3.0.2)
project(stereo_camera_ros)


set(CMAKE_BUILD_TYPE Debug) #RelWithDebInfo

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
)

## Find system dependencies
find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stereo_camera_ros
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport camera_info_manager cv_bridge dynamic_reconfigure
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIR}
)

# Add stereo_camera library
add_library(stereo_camera
  src/stereo_camera.cpp
)
target_include_directories(stereo_camera PUBLIC include/${PROJECT_NAME})
target_link_libraries(stereo_camera
  uvc
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
  ${JPEG_LIBRARIES}
)

# Add the ROS wrapper node
add_executable(stereo_camera_node 
  src/stereo_camera_node.cpp
  src/frame_processor.cpp
)
target_include_directories(stereo_camera_node PUBLIC include/${PROJECT_NAME})

target_link_libraries(stereo_camera_node
  stereo_camera
  ${catkin_LIBRARIES}
)

# 创建双目相机演示程序
add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

#############
## Install ##
#############

install(TARGETS stereo_camera stereo_camera_node stereo_camera_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
