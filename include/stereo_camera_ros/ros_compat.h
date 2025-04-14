#ifndef ROS_COMPAT_H
#define ROS_COMPAT_H

/* 
 * This header file provides a compatibility layer between ROS1 and ROS2
 * Resolves differences between the two versions' APIs through namespace aliases and wrapper functions
 */

// Detect ROS version
#if defined(AMENT_CMAKE) || defined(USING_ROS2)
    #define USE_ROS2
#endif

#ifdef USE_ROS2
    // ROS2 headers
    #include <rclcpp/rclcpp.hpp>
    #include <ament_index_cpp/get_package_share_directory.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <sensor_msgs/msg/camera_info.hpp>
    #include <camera_info_manager/camera_info_manager.hpp>
    #include <image_transport/image_transport.hpp>
    #include <cv_bridge/cv_bridge.h>
    #include <std_msgs/msg/header.hpp>
#else
    // ROS1 headers
    #include <ros/ros.h>
    #include <ros/package.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/CameraInfo.h>
    #include <camera_info_manager/camera_info_manager.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
#endif

namespace ros_compat {

// Basic type definitions
#ifdef USE_ROS2
    using Clock = rclcpp::Clock;
    using Logger = rclcpp::Logger;
    using Node = rclcpp::Node;
    using NodePtr = std::shared_ptr<rclcpp::Node>;
    using Time = rclcpp::Time;
    using Duration = rclcpp::Duration;
    using Timer = rclcpp::TimerBase::SharedPtr;
    namespace msg = sensor_msgs::msg;
#else
    using Clock = ros::Time;  // ROS1 doesn't have a separate Clock class, using Time instead
    using Logger = std::string;  // ROS1 uses string as log category
    using Node = ros::NodeHandle;
    using NodePtr = ros::NodeHandle;
    using Time = ros::Time;
    using Duration = ros::Duration;
    using Timer = ros::Timer;
    namespace msg = sensor_msgs;
#endif

// Message type definitions
#ifdef USE_ROS2
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using CameraInfoPtr = std::shared_ptr<CameraInfo>;
    using Image = sensor_msgs::msg::Image;
    using ImagePtr = std::shared_ptr<Image>;
    using Header = std_msgs::msg::Header;
#else
    using CameraInfo = sensor_msgs::CameraInfo;
    using CameraInfoPtr = sensor_msgs::CameraInfoPtr;
    using Image = sensor_msgs::Image;
    using ImagePtr = sensor_msgs::ImagePtr;
    using Header = std_msgs::Header;
#endif

// Utility class definitions
using CameraInfoManager = camera_info_manager::CameraInfoManager;

// Unified publisher class
#ifdef USE_ROS2
    template<typename T>
    using Publisher = typename rclcpp::Publisher<T>::SharedPtr;
#else
    template<typename T>
    using Publisher = ros::Publisher;
#endif

// Image transport publisher
#ifdef USE_ROS2
    using ImagePublisher = Publisher<sensor_msgs::msg::Image>;
#else
    using ImagePublisher = image_transport::Publisher;
#endif

// Initialization function
inline void init(int argc, char** argv, const std::string& node_name) {
#ifdef USE_ROS2
    rclcpp::init(argc, argv);
#else
    ros::init(argc, argv, node_name);
#endif
}

// Shutdown function
inline void shutdown() {
#ifdef USE_ROS2
    rclcpp::shutdown();
#else
    ros::shutdown();
#endif
}

// Message loop - using namespace qualification to clearly distinguish between ROS1 and ROS2 versions, avoiding conflicts
#ifdef USE_ROS2
inline void ros_spin(NodePtr node) {
    rclcpp::spin(node);
}
#else
inline void ros_spin() {
    ros::spin();
}
#endif

// Create node
#ifdef USE_ROS2
inline NodePtr create_node(const std::string& name, const std::string& ns = "") {
    if (ns.empty()) {
        return std::make_shared<rclcpp::Node>(name);
    } else {
        return std::make_shared<rclcpp::Node>(name, ns);
    }
}
#else
inline NodePtr create_node(const std::string& name, const std::string& ns = "") {
    if (ns.empty()) {
        return ros::NodeHandle(name);
    } else {
        return ros::NodeHandle(ns + "/" + name);
    }
}
#endif

// Get current time
inline Time now() {
#ifdef USE_ROS2
    return rclcpp::Clock().now();
#else
    return ros::Time::now();
#endif
}

// Log functions
#ifdef USE_ROS2
inline void log_info(const NodePtr& node, const std::string& message) {
    RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
}

inline void log_warn(const NodePtr& node, const std::string& message) {
    RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
}

inline void log_error(const NodePtr& node, const std::string& message) {
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
}

inline void log_debug(const NodePtr& node, const std::string& message) {
    RCLCPP_DEBUG(node->get_logger(), "%s", message.c_str());
}
#else
inline void log_info(const NodePtr&, const std::string& message) {
    ROS_INFO("%s", message.c_str());
}

inline void log_warn(const NodePtr&, const std::string& message) {
    ROS_WARN("%s", message.c_str());
}

inline void log_error(const NodePtr&, const std::string& message) {
    ROS_ERROR("%s", message.c_str());
}

inline void log_debug(const NodePtr&, const std::string& message) {
    ROS_DEBUG("%s", message.c_str());
}
#endif

// Create publisher
#ifdef USE_ROS2
template<typename T>
inline Publisher<T> create_publisher(NodePtr node, const std::string& topic, int queue_size) {
    return node->create_publisher<T>(topic, queue_size);
}
#else
template<typename T>
inline Publisher<T> create_publisher(NodePtr& node, const std::string& topic, int queue_size) {
    return node.advertise<T>(topic, queue_size);
}
#endif

// Create image publisher
#ifdef USE_ROS2
inline ImagePublisher create_image_publisher(NodePtr node, const std::string& topic, int queue_size) {
    return node->create_publisher<sensor_msgs::msg::Image>(topic, queue_size);
}
#else
inline ImagePublisher create_image_publisher(NodePtr& node, const std::string& topic, int queue_size) {
    image_transport::ImageTransport it(node);
    return it.advertise(topic, queue_size);
}
#endif

// Publish message
#ifdef USE_ROS2
template<typename T>
inline void publish(const Publisher<T>& pub, T& msg) {
    pub->publish(msg);
}
#else
template<typename T1, typename T2>
inline void publish(const T1& pub, const T2& msg) {
    pub.publish(msg);
}
#endif

// Get parameter
#ifdef USE_ROS2
template<typename T>
inline bool get_param(NodePtr node, const std::string& name, T& value) {
    try {
        if (!node->has_parameter(name)) {
            node->declare_parameter(name, value);
        }
        value = node->get_parameter(name).get_value<T>();
        return true;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(node->get_logger(), "Parameter %s not declared: %s", name.c_str(), e.what());
        return false;
    }
}
#else
template<typename T>
inline bool get_param(NodePtr& node, const std::string& name, T& value) {
    return node.getParam(name, value);
}
#endif

// Create timer
#ifdef USE_ROS2
inline Timer create_wall_timer(
    NodePtr node,
    Duration period,
    std::function<void()> callback)
{
    return node->create_wall_timer(std::chrono::nanoseconds(period.nanoseconds()), callback);
}
#else
class TimerCallback {
public:
    explicit TimerCallback(std::function<void()> cb) : callback(cb) {}
    
    void operator()(const ros::TimerEvent&) {
        callback();
    }
    
private:
    std::function<void()> callback;
};

inline Timer create_wall_timer(
    NodePtr& node,
    ros::Duration period,
    std::function<void()> callback)
{
    return node.createTimer(period, TimerCallback(callback));
}
#endif

// Get package path
inline std::string get_package_share_directory(const std::string& package_name) {
#ifdef USE_ROS2
    return ament_index_cpp::get_package_share_directory(package_name);
#else
    return ros::package::getPath(package_name);
#endif
}

// Get camera manager node pointer
#ifdef USE_ROS2
inline rclcpp::Node* get_camera_info_node(NodePtr node) {
    return node.get();
}
#else
inline ros::NodeHandle get_camera_info_node(NodePtr& node) {
    return node;
}
#endif

} // namespace ros_compat

#endif // ROS_COMPAT_H