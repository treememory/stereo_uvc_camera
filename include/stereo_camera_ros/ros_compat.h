#ifndef ROS_COMPAT_H
#define ROS_COMPAT_H

/* 
 * 该头文件提供ROS1和ROS2之间的兼容层
 * 通过命名空间别名和包装函数解决两个版本API的差异
 */

// 检测ROS版本
#if defined(AMENT_CMAKE) || defined(USING_ROS2)
    #define USE_ROS2
#endif

#ifdef USE_ROS2
    // ROS2头文件
    #include <rclcpp/rclcpp.hpp>
    #include <ament_index_cpp/get_package_share_directory.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <sensor_msgs/msg/camera_info.hpp>
    #include <camera_info_manager/camera_info_manager.hpp>
    #include <image_transport/image_transport.hpp>
    #include <cv_bridge/cv_bridge.h>
    #include <std_msgs/msg/header.hpp>
#else
    // ROS1头文件
    #include <ros/ros.h>
    #include <ros/package.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/CameraInfo.h>
    #include <camera_info_manager/camera_info_manager.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
#endif

namespace ros_compat {

// 基本类型定义
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
    using Clock = ros::Time;  // ROS1没有独立的Clock类，用Time代替
    using Logger = std::string;  // ROS1使用字符串作为日志类别
    using Node = ros::NodeHandle;
    using NodePtr = ros::NodeHandle;
    using Time = ros::Time;
    using Duration = ros::Duration;
    using Timer = ros::Timer;
    namespace msg = sensor_msgs;
#endif

// 消息类型定义
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

// 工具类定义
using CameraInfoManager = camera_info_manager::CameraInfoManager;

// 统一的发布者类
#ifdef USE_ROS2
    template<typename T>
    using Publisher = typename rclcpp::Publisher<T>::SharedPtr;
#else
    template<typename T>
    using Publisher = ros::Publisher;
#endif

// 图像传输发布者
#ifdef USE_ROS2
    using ImagePublisher = Publisher<sensor_msgs::msg::Image>;
#else
    using ImagePublisher = image_transport::Publisher;
#endif

// 初始化函数
inline void init(int argc, char** argv, const std::string& node_name) {
#ifdef USE_ROS2
    rclcpp::init(argc, argv);
#else
    ros::init(argc, argv, node_name);
#endif
}

// 关闭函数
inline void shutdown() {
#ifdef USE_ROS2
    rclcpp::shutdown();
#else
    ros::shutdown();
#endif
}

// 消息循环 - 使用命名空间限定明确区分ROS1和ROS2的版本，避免冲突
#ifdef USE_ROS2
inline void ros_spin(NodePtr node) {
    rclcpp::spin(node);
}
#else
inline void ros_spin() {
    ros::spin();
}
#endif

// 创建节点
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

// 获取当前时间
inline Time now() {
#ifdef USE_ROS2
    return rclcpp::Clock().now();
#else
    return ros::Time::now();
#endif
}

// 日志函数
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

// 创建发布者
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

// 创建图像发布者
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

// 发布消息
#ifdef USE_ROS2
template<typename T>
inline void publish(const Publisher<T>& pub, T& msg) {
    pub->publish(msg);
}
#else
template<typename T>
inline void publish(const Publisher<T>& pub, const std::shared_ptr<T>& msg) {
    pub.publish(msg);
}
#endif

// 获取参数
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

// 创建定时器
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

// 获取包路径
inline std::string get_package_share_directory(const std::string& package_name) {
#ifdef USE_ROS2
    return ament_index_cpp::get_package_share_directory(package_name);
#else
    return ros::package::getPath(package_name);
#endif
}

// 获取相机管理器所需的节点指针
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