#ifndef STEREO_CAMERA_NODE_H
#define STEREO_CAMERA_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

#include "stereo_camera_ros/stereo_camera.hpp"

// Frame packet structure to hold frame data and metadata
struct FramePacket {
    cv::Mat left_image;
    cv::Mat right_image;
    uint64_t timestamp;
    ros::Time ros_time;
};

class StereoCameraNode {
public:
    StereoCameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~StereoCameraNode();

private:
    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    
    // Camera parameters
    int device_index_;
    int width_;
    int height_;
    int fps_;
    bool use_mjpeg_;
    float split_ratio_;
    std::string left_camera_name_;
    std::string right_camera_name_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
    std::string left_image_topic_;
    std::string right_image_topic_;
    
    // ROS publishers
    image_transport::Publisher left_img_pub_;
    image_transport::Publisher right_img_pub_;
    ros::Publisher left_info_pub_;
    ros::Publisher right_info_pub_;
    
    // Camera info managers
    boost::shared_ptr<camera_info_manager::CameraInfoManager> left_camera_info_manager_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> right_camera_info_manager_;
    
    // Camera
    std::unique_ptr<StereoCamera> camera_;
    
    // Status timer
    ros::Timer timer_;
    ros::Timer process_timer_;
    
    // Threading components
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> processing_thread_active_;
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    boost::circular_buffer<FramePacket> frame_buffer_;
    
    // Performance metrics
    std::atomic<uint64_t> dropped_frames_;
    std::atomic<uint64_t> processed_frames_;
    
    // Statistics
    ros::Time last_frame_time_;
    int frame_count_;
    double current_fps_;
    
    // Frame processing methods
    void frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data);
    void startProcessingThread();
    void processFrames();
    void publishFrame(const FramePacket& packet);
    
    // Configuration and status methods
    void timerCallback(const ros::TimerEvent& event);
};

#endif // STEREO_CAMERA_NODE_H
