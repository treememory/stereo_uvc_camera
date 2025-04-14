#include "stereo_camera_ros/stereo_camera_node.h"

void StereoCameraNode::frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data) {
    Time ros_stamp = now();
    
    // Check if images are valid
    if (left.empty() || right.empty()) {
        logWarn("Received empty image frame");
        return;
    }
    
    // Create frame packet, containing deep copies of images
    FramePacket packet;
    try {
        if (!left.empty() && !right.empty()) {
            // left.copyTo(packet.left_image);
            // right.copyTo(packet.right_image);
            packet.left_image = left;
            packet.right_image = right;
            packet.timestamp = timestamp;
            // Convert nanosecond timestamp to Time
            uint32_t sec = timestamp / 1000000000ULL;
            uint32_t nsec = timestamp % 1000000000ULL;
            packet.capture_time = Time(sec, nsec);
            packet.receive_time = now();
        } else {
            logError("Unable to copy images: left image empty=" + std::string(left.empty() ? "yes" : "no") + 
                     ", right image empty=" + std::string(right.empty() ? "yes" : "no"));
            return;
        }
    } catch (const cv::Exception& e) {
        logError("OpenCV exception (copying images): " + std::string(e.what()));
        return;
    }
    
    // Add frame to buffer
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        // If buffer is full, discard oldest frame
        if (frame_buffer_.full()) {
            dropped_frames_++;
            frame_buffer_.pop_front();
        }
        
        // Add new frame
        frame_buffer_.push_back(std::move(packet));
    }
    
    // Notify processing thread
    frame_condition_.notify_one();
    
    // If processing thread is not running, start it
    if (!processing_thread_active_) {
        startProcessingThread();
    }
}

void StereoCameraNode::startProcessingThread() {
    if (!processing_thread_active_) {
        running_ = true;
        processing_thread_active_ = true;
        processing_thread_ = std::thread(&StereoCameraNode::processFrames, this);
    }
}

void StereoCameraNode::processFrames() {
    logInfo("Frame processing thread started");
    
    while (running_) {
        FramePacket packet;
        bool has_frame = false;
        
        // Get next frame from buffer
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            
            // Wait for frame or until shutdown
            if (frame_buffer_.empty()) {
                frame_condition_.wait_for(lock, std::chrono::milliseconds(100), 
                                        [this]() { return !frame_buffer_.empty() || !running_; });
            }
            
            // Check if there's a frame to process
            if (!frame_buffer_.empty() && running_) {
                packet = std::move(frame_buffer_.front());
                frame_buffer_.pop_front();
                has_frame = true;
            }
        }
        
        // If there's a frame and still running, process it
        if (has_frame && running_) {
            publishFrame(packet);
            processed_frames_++;
        }
    }
    
    processing_thread_active_ = false;
    logInfo("Frame processing thread stopped");
}

void StereoCameraNode::publishFrame(const FramePacket& packet) {
    // Check image validity
    if (packet.left_image.empty() || packet.right_image.empty()) {
        logWarn("Received empty image when publishing frame");
        return;
    }
    
    // Get camera info
    CameraInfoPtr left_info, right_info;
    try {
        left_info.reset(new CameraInfo(left_camera_info_manager_->getCameraInfo()));
        right_info.reset(new CameraInfo(right_camera_info_manager_->getCameraInfo()));
    } catch (const std::exception& e) {
        logError("Error getting camera info: " + std::string(e.what()));
        return;
    }
    
    // Set timestamp and frame ID
    left_info->header.stamp = packet.capture_time;
    left_info->header.frame_id = left_frame_id_;
    right_info->header.stamp = packet.capture_time;
    right_info->header.frame_id = right_frame_id_;
    
    // Process and publish left camera image and info
    try {
        // Convert OpenCV image to ROS message
        cv_bridge::CvImage left_img_bridge;
        left_img_bridge.encoding = "bgr8";
        left_img_bridge.image = packet.left_image;
        left_img_bridge.header.stamp = packet.capture_time;
        left_img_bridge.header.frame_id = left_frame_id_;
        
        // Publish image
        try {
            ImagePtr left_img_msg = left_img_bridge.toImageMsg();
            publish(left_img_pub_, *left_img_msg);
        } catch (const std::exception& e) {
            logError("Exception publishing left image: " + std::string(e.what()));
        }
        
        // Publish camera info
        publish(left_info_pub_, *left_info);
    } catch (const cv::Exception& e) {
        logError("OpenCV exception processing left image: " + std::string(e.what()));
    } catch (const std::exception& e) {
        logError("Exception processing left image: " + std::string(e.what()));
    }
    
    // Process and publish right camera image and info
    try {
        // Convert OpenCV image to ROS message
        cv_bridge::CvImage right_img_bridge;
        right_img_bridge.encoding = "bgr8";
        right_img_bridge.image = packet.right_image;
        right_img_bridge.header.stamp = packet.capture_time;
        right_img_bridge.header.frame_id = right_frame_id_;
        
        // Publish image
        try {
            ImagePtr right_img_msg = right_img_bridge.toImageMsg();
            publish(right_img_pub_, *right_img_msg);
        } catch (const std::exception& e) {
            logError("Exception publishing right image: " + std::string(e.what()));
        }
        
        // Publish camera info
        publish(right_info_pub_, *right_info);
    } catch (const cv::Exception& e) {
        logError("OpenCV exception processing right image: " + std::string(e.what()));
    } catch (const std::exception& e) {
        logError("Exception processing right image: " + std::string(e.what()));
    }

    // Statistics: ROS publishing frame rate, recent publishing delay, and delay for each frame now()-receive_time
#ifdef USE_ROS2
    frame_timestamp_history_[packet.timestamp] = {packet.receive_time.nanoseconds(), now().nanoseconds()};
#else
    frame_timestamp_history_[packet.timestamp] = {packet.receive_time.toNSec(), now().toNSec()};
#endif
    double dt = (frame_timestamp_history_.rbegin()->first - frame_timestamp_history_.begin()->first) * 1e-9;
    if(dt>1.0){
        current_fps_ = (frame_timestamp_history_.size()-1) / dt;
        double sum_delay1 = 0, sum_delay2 = 0;
        for (const auto& it : frame_timestamp_history_) {
            sum_delay1 += (it.second.first - it.first) * 1e-9;
            sum_delay2 += (it.second.second - it.second.first) * 1e-9;
        }
        double avg_delay1 = sum_delay1 / frame_timestamp_history_.size();
        double avg_delay2 = sum_delay2 / frame_timestamp_history_.size();
        logInfo("Publishing statistics: Image: [" + std::to_string(packet.left_image.cols) + "x" + std::to_string(packet.left_image.rows) + "], " +
                " Current frame rate=" + std::to_string(current_fps_) + 
                " fps, Set frame rate=" + std::to_string(fps_) +
                " fps, Average driver delay=" + std::to_string(avg_delay1 * 1000) + 
                " ms, Average publishing delay=" + std::to_string(avg_delay2 * 1000) + 
                " ms");
        frame_timestamp_history_.clear();
    }

}
