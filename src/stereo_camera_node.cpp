#include "stereo_camera_ros/stereo_camera_node.h"


StereoCameraNode::StereoCameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh), it_(nh),
          frame_buffer_(10), // Circular buffer with capacity for 10 frames
          running_(false), processing_thread_active_(false),
          dropped_frames_(0), processed_frames_(0) {
        
        // Load parameters
        private_nh_.param<int>("device_index", device_index_, 0);
        private_nh_.param<int>("width", width_, 3840);
        private_nh_.param<int>("height", height_, 1080);
        private_nh_.param<int>("fps", fps_, 60);
        private_nh_.param<bool>("use_mjpeg", use_mjpeg_, true);
        private_nh_.param<float>("split_ratio", split_ratio_, 0.5f);
        private_nh_.param<std::string>("left_camera_name", left_camera_name_, "left_camera");
        private_nh_.param<std::string>("right_camera_name", right_camera_name_, "right_camera");
        private_nh_.param<std::string>("left_frame_id", left_frame_id_, "left_camera_optical_frame");
        private_nh_.param<std::string>("right_frame_id", right_frame_id_, "right_camera_optical_frame");
        private_nh_.param<std::string>("left_camera_info_url", left_camera_info_url_, "");
        private_nh_.param<std::string>("right_camera_info_url", right_camera_info_url_, "");
        private_nh_.param<std::string>("left_image_topic", left_image_topic_, "left/image_raw");
        private_nh_.param<std::string>("right_image_topic", right_image_topic_, "right/image_raw");
        
        // Get package path for calibration files
        std::string pkg_path = ros::package::getPath("stereo_camera_ros");
        std::string left_calib_path = "file://" + pkg_path + "/config/left_camera.yaml";
        std::string right_calib_path = "file://" + pkg_path + "/config/right_camera.yaml";
        
        // Override camera_info_url if not specified
        if (left_camera_info_url_.empty()) {
            left_camera_info_url_ = left_calib_path;
            ROS_INFO("Using left camera calibration from: %s", left_camera_info_url_.c_str());
        }
        
        if (right_camera_info_url_.empty()) {
            right_camera_info_url_ = right_calib_path;
            ROS_INFO("Using right camera calibration from: %s", right_camera_info_url_.c_str());
        }
        
        // Initialize camera info managers
        left_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
            ros::NodeHandle(nh_, "left"), left_camera_name_, left_camera_info_url_));
        right_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
            ros::NodeHandle(nh_, "right"), right_camera_name_, right_camera_info_url_));
        
        // Set up image publishers
        left_img_pub_ = it_.advertise(left_image_topic_, 1);
        right_img_pub_ = it_.advertise(right_image_topic_, 1);
        
        // Set up camera info publishers with separate topics
        left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info/left", 1);
        right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info/right", 1);
        
        // Initialize camera
        camera_.reset(new StereoCamera());
        camera_->setStereoSplitRatio(split_ratio_);
        
        // Open camera
        ROS_INFO("Opening camera with params: width=%d, height=%d, fps=%d, mjpeg=%s, device_index=%d",
                width_, height_, fps_, use_mjpeg_ ? "true" : "false", device_index_);
        
        if (!camera_->openWithParams(width_, height_, fps_, use_mjpeg_, device_index_)) {
            ROS_ERROR("Failed to open camera!");
            ros::shutdown();
            return;
        }
        
        ROS_INFO("Camera opened successfully");
        camera_->printDeviceInfo();
        
        // Set frame callback
        camera_->setFrameCallback(
            std::bind(&StereoCameraNode::frameCallback, this, 
                      std::placeholders::_1, std::placeholders::_2, 
                      std::placeholders::_3, std::placeholders::_4),
            this);
        
        // Create timer for checking camera status
        timer_ = nh_.createTimer(ros::Duration(1.0), &StereoCameraNode::timerCallback, this);
        
        ROS_INFO("Stereo camera node initialized.");
    }
    
    StereoCameraNode::~StereoCameraNode() {
        // Stop processing thread
        running_ = false;
        frame_condition_.notify_all();
        
        // Wait for processing thread to finish
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        // Close camera
        if (camera_) {
            camera_->close();
        }
        
        ROS_INFO("Stereo camera node shutdown. Stats: Processed %ld frames, Dropped %ld frames", 
                 processed_frames_.load(), dropped_frames_.load());
    }
    
    void StereoCameraNode::timerCallback(const ros::TimerEvent& event) {
        if (camera_ && camera_->isOpened()) {
            ROS_DEBUG("Camera running at %.1f fps", current_fps_);
        } else {
            ROS_WARN("Camera is not opened!");
        }
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    StereoCameraNode node(nh, private_nh);
    
    ros::spin();
    
    return 0;
}
