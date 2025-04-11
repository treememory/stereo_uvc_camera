#include "stereo_camera_ros/stereo_camera_node.h"

void StereoCameraNode::frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data) {
    ros::Time ros_stamp = ros::Time::now();
    
    // Check if images are valid
    if (left.empty() || right.empty()) {
        ROS_WARN("接收到空图像帧");
        return;
    }
    
    // Update FPS statistics
    if (last_frame_time_ != ros::Time(0)) {
        double dt = (ros_stamp - last_frame_time_).toSec();
        frame_count_++;
        if (dt >= 1.0) {
            current_fps_ = frame_count_ / dt;
            frame_count_ = 0;
            last_frame_time_ = ros_stamp;
        }
    } else {
        last_frame_time_ = ros_stamp;
        frame_count_ = 1;
    }
    
    // Convert timestamp from nanoseconds to ros::Time
    uint32_t sec = timestamp / 1000000000ULL;
    uint32_t nsec = timestamp % 1000000000ULL;
    ros::Time frame_time(sec, nsec);
    
    // Create frame packet with deep copies of the images
    FramePacket packet;
    try {
        if (!left.empty() && !right.empty()) {
            left.copyTo(packet.left_image);
            right.copyTo(packet.right_image);
            packet.timestamp = timestamp;
            packet.ros_time = frame_time;
        } else {
            ROS_ERROR("无法复制图像：左图像为空=%s, 右图像为空=%s", 
                     left.empty() ? "是" : "否", right.empty() ? "是" : "否");
            return;
        }
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV异常（复制图像）: %s", e.what());
        return;
    }
    
    // Add frame to the buffer
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        // If buffer is full, drop oldest frame
        if (frame_buffer_.full()) {
            dropped_frames_++;
            frame_buffer_.pop_front();
        }
        
        // Add new frame
        frame_buffer_.push_back(std::move(packet));
    }
    
    // Notify processing thread
    frame_condition_.notify_one();
    
    // Start processing thread if not already running
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
    ROS_INFO("Frame processing thread started");
    
    while (running_) {
        FramePacket packet;
        bool has_frame = false;
        
        // Get next frame from buffer
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            
            // Wait for a frame or until shutdown
            if (frame_buffer_.empty()) {
                frame_condition_.wait_for(lock, std::chrono::milliseconds(100), 
                                        [this]() { return !frame_buffer_.empty() || !running_; });
            }
            
            // Check if we have a frame to process
            if (!frame_buffer_.empty() && running_) {
                packet = std::move(frame_buffer_.front());
                frame_buffer_.pop_front();
                has_frame = true;
            }
        }
        
        // Process the frame if we have one
        if (has_frame && running_) {
            publishFrame(packet);
            processed_frames_++;
        }
    }
    
    processing_thread_active_ = false;
    ROS_INFO("Frame processing thread stopped");
}

void StereoCameraNode::publishFrame(const FramePacket& packet) {

    // 检查图像有效性
    if (packet.left_image.empty() || packet.right_image.empty()) {
        ROS_WARN("发布帧时收到空图像");
        return;
    }
    
    // 获取相机信息
    sensor_msgs::CameraInfoPtr left_info, right_info;
    try {
        left_info.reset(new sensor_msgs::CameraInfo(left_camera_info_manager_->getCameraInfo()));
        right_info.reset(new sensor_msgs::CameraInfo(right_camera_info_manager_->getCameraInfo()));
    } catch (const std::exception& e) {
        ROS_ERROR("获取相机信息时发生错误: %s", e.what());
        return;
    }
    
    // 设置时间戳和帧ID
    left_info->header.stamp = packet.ros_time;
    left_info->header.frame_id = left_frame_id_;
    right_info->header.stamp = packet.ros_time;
    right_info->header.frame_id = right_frame_id_;
    
    // Publish left image and camera info if there are subscribers
    if (left_img_pub_.getNumSubscribers() > 0 || left_info_pub_.getNumSubscribers() > 0) {
        try {
            // Convert OpenCV image to ROS message
            cv_bridge::CvImage left_img_bridge(std_msgs::Header(), "bgr8", packet.left_image);
            left_img_bridge.header.stamp = packet.ros_time;
            left_img_bridge.header.frame_id = left_frame_id_;
            
            // Publish the image
            if (left_img_pub_.getNumSubscribers() > 0) {
                try {
                    sensor_msgs::ImagePtr left_img_msg = left_img_bridge.toImageMsg();
                    left_img_pub_.publish(left_img_msg);
                } catch (const std::exception& e) {
                    ROS_ERROR("发布左图像时异常: %s", e.what());
                }
            }
            
            // Always publish camera info
            left_info_pub_.publish(left_info);
        } catch (const cv::Exception& e) {
            ROS_ERROR("处理左图像时OpenCV异常: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("处理左图像时异常: %s", e.what());
        }
    }
    
    // Publish right image and camera info if there are subscribers
    if (right_img_pub_.getNumSubscribers() > 0 || right_info_pub_.getNumSubscribers() > 0) {
        try {
            // Convert OpenCV image to ROS message
            cv_bridge::CvImage right_img_bridge(std_msgs::Header(), "bgr8", packet.right_image);
            right_img_bridge.header.stamp = packet.ros_time;
            right_img_bridge.header.frame_id = right_frame_id_;
            
            // Publish the image
            if (right_img_pub_.getNumSubscribers() > 0) {
                try {
                    sensor_msgs::ImagePtr right_img_msg = right_img_bridge.toImageMsg();
                    right_img_pub_.publish(right_img_msg);
                } catch (const std::exception& e) {
                    ROS_ERROR("发布右图像时异常: %s", e.what());
                }
            }
            
            // Always publish camera info
            right_info_pub_.publish(right_info);
        } catch (const cv::Exception& e) {
            ROS_ERROR("处理右图像时OpenCV异常: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("处理右图像时异常: %s", e.what());
        }
    }
}
