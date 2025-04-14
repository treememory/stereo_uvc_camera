cmake_minimum_required(VERSION 3.10.0)
project(stereo_camera_ros)

# Determine ROS version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
  message(STATUS "Using ROS_VERSION=$ENV{ROS_VERSION} from environment")
elseif(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} MATCHES "noetic|melodic|kinetic")
    set(ROS_VERSION 1)
  else()
    set(ROS_VERSION 2)
  endif()
  message(STATUS "Detected ROS_VERSION=${ROS_VERSION} from ROS_DISTRO=$ENV{ROS_DISTRO}")
else()
  # Default to No ROS
  set(ROS_VERSION 0)
  message(STATUS "No ROS_VERSION or ROS_DISTRO set; defaulting to ROS_VERSION=0")
endif()

# Configure build type
set(CMAKE_BUILD_TYPE Release) #RelWithDebInfo

# Set compiler options based on ROS version
if(${ROS_VERSION} EQUAL 1)
  message(STATUS "Building for ROS1")
  
  # ROS1 build settings
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
  )

  # System dependencies
  find_package(OpenCV REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(Threads REQUIRED)

  # Catkin specific configuration
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES stereo_camera_ros
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport camera_info_manager cv_bridge dynamic_reconfigure
    DEPENDS OpenCV
  )

  # Include directories
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
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JPEG_LIBRARIES}
    uvc
    usb-1.0  # Add explicit libusb dependency and adjust order
  )

  # Add ROS wrapper node
  add_executable(stereo_camera_node 
    src/stereo_camera_node.cpp
    src/frame_processor.cpp
  )
  target_include_directories(stereo_camera_node PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera_node
    stereo_camera
    ${catkin_LIBRARIES}
  )

  # Stereo camera demo program
  add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
  target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

  # Installation rules
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

elseif(${ROS_VERSION} EQUAL 2)
  message(STATUS "Building for ROS2")
  
  # Set C++ standard
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  
  # ROS2 build settings
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(Threads REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # Define ROS2 compilation flags
  add_definitions(-DUSING_ROS2)

  # Include directories
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIR}
  )

  # Add stereo_camera library
  add_library(stereo_camera
    src/stereo_camera.cpp
  )
  target_include_directories(stereo_camera PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JPEG_LIBRARIES}
    uvc
    usb-1.0  # Add explicit libusb dependency and adjust order
  )

  # Add ROS2 wrapper node
  add_executable(stereo_camera_node 
    src/stereo_camera_node.cpp
    src/frame_processor.cpp
  )
  target_include_directories(stereo_camera_node PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera_node
    stereo_camera
  )
  ament_target_dependencies(stereo_camera_node
    rclcpp
    sensor_msgs
    std_msgs
    image_transport
    camera_info_manager
    cv_bridge
    ament_index_cpp
  )

  # Stereo camera demo program
  add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
  target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

  # Installation rules
  install(TARGETS 
    stereo_camera 
    stereo_camera_node 
    stereo_camera_demo
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  # Export dependencies
  ament_export_include_directories(include)
  ament_export_libraries(stereo_camera)
  ament_export_dependencies(
    rclcpp
    sensor_msgs
    std_msgs
    image_transport
    camera_info_manager
    cv_bridge
    OpenCV
  )

  ament_package()
else()
  # System dependencies
  find_package(OpenCV REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(Threads REQUIRED)

  # Include directories
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIR}
  )

  # Add stereo_camera library
  add_library(stereo_camera
    src/stereo_camera.cpp
  )
  target_include_directories(stereo_camera PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JPEG_LIBRARIES}
    uvc
  )

  add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
  target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
endif()
