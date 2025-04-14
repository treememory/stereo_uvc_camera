#ifndef STEREO_CAMERA_NODE_H
#define STEREO_CAMERA_NODE_H

#include "stereo_camera_ros/ros_compat.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

#include "stereo_camera_ros/stereo_camera.hpp"

using namespace ros_compat;

// Frame packet structure to hold frame data and metadata
struct FramePacket {
    cv::Mat left_image;
    cv::Mat right_image;
    uint64_t timestamp;
    Time capture_time;
    Time receive_time;
};

class StereoCameraNode {
public:
    StereoCameraNode(NodePtr& main_node, NodePtr& private_node);
    ~StereoCameraNode();

private:
    // Node handles
    NodePtr node_;
    NodePtr private_node_;
    
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
    ImagePublisher left_img_pub_;
    ImagePublisher right_img_pub_;
    Publisher<CameraInfo> left_info_pub_;
    Publisher<CameraInfo> right_info_pub_;
    
    // Camera info managers
    std::shared_ptr<CameraInfoManager> left_camera_info_manager_;
    std::shared_ptr<CameraInfoManager> right_camera_info_manager_;
    
    // Camera
    std::unique_ptr<StereoCamera> camera_;
    
    // Status timer
    Timer timer_;
    
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

    std::map<uint64_t, std::pair<uint64_t, uint64_t>> frame_timestamp_history_;
    
    // Statistics
    Time last_frame_time_;
    int frame_count_;
    double current_fps_;
    
    // Frame processing methods
    void frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data);
    void startProcessingThread();
    void processFrames();
    void publishFrame(const FramePacket& packet);
    
    // Configuration and status methods
    void timerCallback();
    
    // Logging helper
    void logInfo(const std::string& msg);
    void logWarn(const std::string& msg);
    void logError(const std::string& msg);
    void logDebug(const std::string& msg);
};

#endif // STEREO_CAMERA_NODE_H
