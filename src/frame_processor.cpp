#include "stereo_camera_ros/stereo_camera_node.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>  // For visualization purposes

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

bool StereoCameraNode::initRectificationMaps() {
    // Check if camera info is available
    if (!left_camera_info_manager_ || !right_camera_info_manager_) {
        logError("Camera info managers not initialized");
        return false;
    }

    try {
        // Get camera info
        CameraInfo left_info = left_camera_info_manager_->getCameraInfo();
        CameraInfo right_info = right_camera_info_manager_->getCameraInfo();

        if (left_info.width == 0 || left_info.height == 0 || right_info.width == 0 || right_info.height == 0) {
            logError("Invalid camera info dimensions");
            return false;
        }

        // Get camera matrices and distortion coefficients from camera info
        cv::Mat K1(3, 3, CV_64F);
        cv::Mat K2(3, 3, CV_64F);
        cv::Mat D1, D2;
        cv::Mat R1, R2, P1, P2;

        // Extract camera matrices
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K1.at<double>(i, j) = left_info.K[i * 3 + j];
                K2.at<double>(i, j) = right_info.K[i * 3 + j];
            }
        }

        // Extract distortion coefficients
        D1 = cv::Mat(1, left_info.D.size(), CV_64F);
        D2 = cv::Mat(1, right_info.D.size(), CV_64F);
        for (size_t i = 0; i < left_info.D.size(); i++) {
            D1.at<double>(0, i) = left_info.D[i];
        }
        for (size_t i = 0; i < right_info.D.size(); i++) {
            D2.at<double>(0, i) = right_info.D[i];
        }

        // Extract rectification matrices
        R1 = cv::Mat(3, 3, CV_64F);
        R2 = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R1.at<double>(i, j) = left_info.R[i * 3 + j];
                R2.at<double>(i, j) = right_info.R[i * 3 + j];
            }
        }

        // Extract projection matrices
        P1 = cv::Mat(3, 4, CV_64F);
        P2 = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                P1.at<double>(i, j) = left_info.P[i * 4 + j];
                P2.at<double>(i, j) = right_info.P[i * 4 + j];
            }
        }

        // Create rectification maps
        cv::Size img_size(left_info.width, left_info.height);

        logInfo("Initializing rectification maps for " + std::to_string(img_size.width) + "x" + 
                std::to_string(img_size.height) + " images");

        // Initialize rectification maps
        cv::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_32FC1, left_map1_, left_map2_);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_32FC1, right_map1_, right_map2_);

        std::cout << "K1=" << K1 << std::endl;
        std::cout << "D1=" << D1 << std::endl;
        std::cout << "R1=" << R1 << std::endl;
        std::cout << "P1=" << P1 << std::endl;
        std::cout << "K2=" << K2 << std::endl;
        std::cout << "D2=" << D2 << std::endl;
        std::cout << "R2=" << R2 << std::endl;
        std::cout << "P2=" << P2 << std::endl;

        logInfo("Rectification maps successfully initialized");
        return true;
    }
    catch (const std::exception& e) {
        logError("Exception while initializing rectification maps: " + std::string(e.what()));
        return false;
    }
}

void StereoCameraNode::publishRectifiedImages(const cv::Mat& left, const cv::Mat& right, 
                                             const Time& timestamp, const std::string& left_frame_id, 
                                             const std::string& right_frame_id) {
    // Check if rectification maps are initialized
    if (!rectification_maps_initialized_) {
        if (!initRectificationMaps()) {
            logWarn("Failed to initialize rectification maps");
            return;
        }
        rectification_maps_initialized_ = true;
    }

    // Rectify images
    cv::Mat left_rect, right_rect;
    try {
        cv::remap(left, left_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(right, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);
    }
    catch (const cv::Exception& e) {
        logError("OpenCV exception during image rectification: " + std::string(e.what()));
        return;
    }

    // Create visualization if enabled
    if (show_rectification_visual_) {
        cv::Mat visual = createRectificationVisualization(left_rect, right_rect);
        cv::imshow("Stereo Rectification Visualization", visual);
        cv::waitKey(1);
    }

    // Create image messages for rectified images
    try {
        // Left rectified image
        cv_bridge::CvImage left_rect_bridge;
        left_rect_bridge.encoding = "bgr8";
        left_rect_bridge.image = left_rect;
        left_rect_bridge.header.stamp = timestamp;
        left_rect_bridge.header.frame_id = left_frame_id;
        
        // Right rectified image
        cv_bridge::CvImage right_rect_bridge;
        right_rect_bridge.encoding = "bgr8";
        right_rect_bridge.image = right_rect;
        right_rect_bridge.header.stamp = timestamp;
        right_rect_bridge.header.frame_id = right_frame_id;
        
        // Publish rectified images
        ImagePtr left_rect_msg = left_rect_bridge.toImageMsg();
        ImagePtr right_rect_msg = right_rect_bridge.toImageMsg();
        
        publish(left_rect_img_pub_, *left_rect_msg);
        publish(right_rect_img_pub_, *right_rect_msg);
    }
    catch (const std::exception& e) {
        logError("Exception while publishing rectified images: " + std::string(e.what()));
    }
}

cv::Mat StereoCameraNode::createRectificationVisualization(const cv::Mat& left_rect, const cv::Mat& right_rect) {
    // Create a visualization that shows both images side by side with horizontal lines
    // to validate that the rectification aligns the epipolar lines
    
    cv::Mat visual;
    
    try {
        // Make sure both images have the same size
        if (left_rect.size() != right_rect.size()) {
            logError("Left and right rectified images have different sizes");
            return cv::Mat();
        }
        
        int height = left_rect.rows;
        int width = left_rect.cols;
        
        // Create an image that combines both left and right images
        visual = cv::Mat(height, width * 2, CV_8UC3);
        cv::Mat left_roi(visual, cv::Rect(0, 0, width, height));
        cv::Mat right_roi(visual, cv::Rect(width, 0, width, height));
        
        left_rect.copyTo(left_roi);
        right_rect.copyTo(right_roi);

        std::vector<cv::Vec3b> colors = {cv::Vec3b(0, 255, 0), cv::Vec3b(255, 0, 0), cv::Vec3b(0, 0, 255), 
                                          cv::Vec3b(255, 255, 0), cv::Vec3b(0, 255, 255)};
        
        // Draw horizontal lines every 20 pixels to check epipolar alignment
        for (int y = 0; y < height; y += 20) {
            cv::line(visual, cv::Point(0, y), cv::Point(width * 2, y), colors[(y/20)%5], 1);
        }
        
        // Draw a vertical line between the two images
        cv::line(visual, cv::Point(width, 0), cv::Point(width, height), cv::Scalar(0, 0, 255), 1);
        
        // Add text labels
        cv::putText(visual, "Left", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(visual, "Right", cv::Point(width + 20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(visual, "Stereo Rectification Visualization", cv::Point(width/2 - 180, height - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    catch (const cv::Exception& e) {
        logError("OpenCV exception creating visualization: " + std::string(e.what()));
        return cv::Mat();
    }
    
    return visual;
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
    
    // Publish rectified images if enabled
    if (enable_rectification_) {
        publishRectifiedImages(packet.left_image, packet.right_image, 
                              packet.capture_time, left_frame_id_, right_frame_id_);
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
