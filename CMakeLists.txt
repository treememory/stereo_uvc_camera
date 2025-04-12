cmake_minimum_required(VERSION 3.10.0)
project(stereo_camera_ros)

# 确定ROS版本
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
  # 默认使用ROS1
  set(ROS_VERSION 1)
  message(STATUS "No ROS_VERSION or ROS_DISTRO set; defaulting to ROS_VERSION=1")
endif()

# 配置构建类型
set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

# 根据ROS版本设置编译选项
if(${ROS_VERSION} EQUAL 1)
  message(STATUS "Building for ROS1")
  
  # ROS1构建设置
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
  )

  # 系统依赖
  find_package(OpenCV REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(Threads REQUIRED)

  # Catkin特定配置
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES stereo_camera_ros
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport camera_info_manager cv_bridge dynamic_reconfigure
    DEPENDS OpenCV
  )

  # 包含目录
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIR}
  )

  # 添加stereo_camera库
  add_library(stereo_camera
    src/stereo_camera.cpp
  )
  target_include_directories(stereo_camera PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JPEG_LIBRARIES}
    uvc
    usb-1.0  # 添加明确的libusb依赖并调整顺序
  )

  # 添加ROS包装节点
  add_executable(stereo_camera_node 
    src/stereo_camera_node.cpp
    src/frame_processor.cpp
  )
  target_include_directories(stereo_camera_node PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera_node
    stereo_camera
    ${catkin_LIBRARIES}
  )

  # 双目相机演示程序
  add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
  target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

  # 安装规则
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

else()
  message(STATUS "Building for ROS2")
  
  # 设置C++标准
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  
  # ROS2构建设置
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

  # 定义ROS2编译标志
  add_definitions(-DUSING_ROS2)

  # 包含目录
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIR}
  )

  # 添加stereo_camera库
  add_library(stereo_camera
    src/stereo_camera.cpp
  )
  target_include_directories(stereo_camera PUBLIC include/${PROJECT_NAME})
  target_link_libraries(stereo_camera
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JPEG_LIBRARIES}
    uvc
    usb-1.0  # 添加明确的libusb依赖并调整顺序
  )

  # 添加ROS2包装节点
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

  # 双目相机演示程序
  add_executable(stereo_camera_demo src/stereo_camera_demo.cpp)
  target_link_libraries(stereo_camera_demo stereo_camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

  # 安装规则
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

  # 导出依赖
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
endif()
