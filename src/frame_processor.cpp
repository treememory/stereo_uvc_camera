#include "stereo_camera_ros/stereo_camera_node.h"

void StereoCameraNode::frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data) {
    Time ros_stamp = now();
    
    // 检查图像是否有效
    if (left.empty() || right.empty()) {
        logWarn("接收到空图像帧");
        return;
    }
    
    // 更新FPS统计信息
#ifdef USE_ROS2
    if (last_frame_time_.seconds() != 0) {
        double dt = (ros_stamp.seconds() - last_frame_time_.seconds());
#else
    if (last_frame_time_ != Time(0)) {
        double dt = (ros_stamp - last_frame_time_).toSec();
#endif
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
    
    // 创建帧数据包，包含图像的深拷贝
    FramePacket packet;
    try {
        if (!left.empty() && !right.empty()) {
            left.copyTo(packet.left_image);
            right.copyTo(packet.right_image);
            packet.timestamp = timestamp;
            // 将纳秒时间戳转换为Time
            uint32_t sec = timestamp / 1000000000ULL;
            uint32_t nsec = timestamp % 1000000000ULL;
            packet.ros_time = Time(sec, nsec);
        } else {
            logError("无法复制图像：左图像为空=" + std::string(left.empty() ? "是" : "否") + 
                     ", 右图像为空=" + std::string(right.empty() ? "是" : "否"));
            return;
        }
    } catch (const cv::Exception& e) {
        logError("OpenCV异常（复制图像）: " + std::string(e.what()));
        return;
    }
    
    // 添加帧到缓冲区
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        // 如果缓冲区已满，丢弃最旧的帧
        if (frame_buffer_.full()) {
            dropped_frames_++;
            frame_buffer_.pop_front();
        }
        
        // 添加新帧
        frame_buffer_.push_back(std::move(packet));
    }
    
    // 通知处理线程
    frame_condition_.notify_one();
    
    // 如果处理线程未运行，则启动它
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
    logInfo("帧处理线程已启动");
    
    while (running_) {
        FramePacket packet;
        bool has_frame = false;
        
        // 从缓冲区获取下一帧
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            
            // 等待帧或直到关闭
            if (frame_buffer_.empty()) {
                frame_condition_.wait_for(lock, std::chrono::milliseconds(100), 
                                        [this]() { return !frame_buffer_.empty() || !running_; });
            }
            
            // 检查是否有帧可处理
            if (!frame_buffer_.empty() && running_) {
                packet = std::move(frame_buffer_.front());
                frame_buffer_.pop_front();
                has_frame = true;
            }
        }
        
        // 如果有帧且仍在运行，则处理帧
        if (has_frame && running_) {
            publishFrame(packet);
            processed_frames_++;
        }
    }
    
    processing_thread_active_ = false;
    logInfo("帧处理线程已停止");
}

void StereoCameraNode::publishFrame(const FramePacket& packet) {
    // 检查图像有效性
    if (packet.left_image.empty() || packet.right_image.empty()) {
        logWarn("发布帧时收到空图像");
        return;
    }
    
    // 获取相机信息
    CameraInfoPtr left_info, right_info;
    try {
        left_info.reset(new CameraInfo(left_camera_info_manager_->getCameraInfo()));
        right_info.reset(new CameraInfo(right_camera_info_manager_->getCameraInfo()));
    } catch (const std::exception& e) {
        logError("获取相机信息时发生错误: " + std::string(e.what()));
        return;
    }
    
    // 设置时间戳和帧ID
    left_info->header.stamp = packet.ros_time;
    left_info->header.frame_id = left_frame_id_;
    right_info->header.stamp = packet.ros_time;
    right_info->header.frame_id = right_frame_id_;
    
    // 处理并发布左相机图像和信息
    try {
        // 将OpenCV图像转换为ROS消息
        cv_bridge::CvImage left_img_bridge;
        left_img_bridge.encoding = "bgr8";
        left_img_bridge.image = packet.left_image;
        left_img_bridge.header.stamp = packet.ros_time;
        left_img_bridge.header.frame_id = left_frame_id_;
        
        // 发布图像
        try {
            ImagePtr left_img_msg = left_img_bridge.toImageMsg();
            publish(left_img_pub_, *left_img_msg);
        } catch (const std::exception& e) {
            logError("发布左图像时异常: " + std::string(e.what()));
        }
        
        // 发布相机信息
        publish(left_info_pub_, *left_info);
    } catch (const cv::Exception& e) {
        logError("处理左图像时OpenCV异常: " + std::string(e.what()));
    } catch (const std::exception& e) {
        logError("处理左图像时异常: " + std::string(e.what()));
    }
    
    // 处理并发布右相机图像和信息
    try {
        // 将OpenCV图像转换为ROS消息
        cv_bridge::CvImage right_img_bridge;
        right_img_bridge.encoding = "bgr8";
        right_img_bridge.image = packet.right_image;
        right_img_bridge.header.stamp = packet.ros_time;
        right_img_bridge.header.frame_id = right_frame_id_;
        
        // 发布图像
        try {
            ImagePtr right_img_msg = right_img_bridge.toImageMsg();
            publish(right_img_pub_, *right_img_msg);
        } catch (const std::exception& e) {
            logError("发布右图像时异常: " + std::string(e.what()));
        }
        
        // 发布相机信息
        publish(right_info_pub_, *right_info);
    } catch (const cv::Exception& e) {
        logError("处理右图像时OpenCV异常: " + std::string(e.what()));
    } catch (const std::exception& e) {
        logError("处理右图像时异常: " + std::string(e.what()));
    }
}
