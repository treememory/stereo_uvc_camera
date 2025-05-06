#include "stereo_camera_ros/stereo_camera_node.h"

StereoCameraNode::StereoCameraNode(NodePtr& main_node, NodePtr& private_node)
    : node_(main_node), private_node_(private_node),
      frame_buffer_(10), // Circular buffer with capacity for 10 frames
      running_(false), processing_thread_active_(false),
      dropped_frames_(0), processed_frames_(0) {
    
    // Load parameters
    device_index_ = 0;
    width_ = 3840;
    height_ = 1080;
    fps_ = 60;
    use_mjpeg_ = true;
    split_ratio_ = 0.5f;
    left_camera_name_ = "left_camera";
    right_camera_name_ = "right_camera";
    left_frame_id_ = "left_camera_optical_frame";
    right_frame_id_ = "right_camera_optical_frame";
    left_camera_info_url_ = "";
    right_camera_info_url_ = "";
    left_image_topic_ = "left/image_raw";
    right_image_topic_ = "right/image_raw";

    enable_rectification_ = true;
    show_rectification_visual_ = true;
    rectification_maps_initialized_ = false;
    
    get_param(private_node_, "device_index", device_index_);
    get_param(private_node_, "width", width_);
    get_param(private_node_, "height", height_);
    get_param(private_node_, "fps", fps_);
    get_param(private_node_, "use_mjpeg", use_mjpeg_);
    get_param(private_node_, "split_ratio", split_ratio_);
    get_param(private_node_, "left_camera_name", left_camera_name_);
    get_param(private_node_, "right_camera_name", right_camera_name_);
    get_param(private_node_, "left_frame_id", left_frame_id_);
    get_param(private_node_, "right_frame_id", right_frame_id_);
    get_param(private_node_, "left_camera_info_url", left_camera_info_url_);
    get_param(private_node_, "right_camera_info_url", right_camera_info_url_);
    get_param(private_node_, "left_image_topic", left_image_topic_);
    get_param(private_node_, "right_image_topic", right_image_topic_);
    get_param(private_node_, "enable_rectification", enable_rectification_);
    get_param(private_node_, "show_rectification_visual", show_rectification_visual_);
    
    // Get package path for calibration files
    std::string pkg_path = get_package_share_directory("stereo_camera_ros");
    std::string left_calib_path = "file://" + pkg_path + "/config/left_camera.yaml";
    std::string right_calib_path = "file://" + pkg_path + "/config/right_camera.yaml";
    
    // If camera info URL is not specified, use default values
    if (left_camera_info_url_.empty()) {
        left_camera_info_url_ = left_calib_path;
        logInfo("Using left camera calibration file: " + left_camera_info_url_);
    }
    
    if (right_camera_info_url_.empty()) {
        right_camera_info_url_ = right_calib_path;
        logInfo("Using right camera calibration file: " + right_camera_info_url_);
    }
    
    // Initialize camera info managers
    auto left_camera_node = get_camera_info_node(private_node_);
    auto right_camera_node = get_camera_info_node(private_node_);
    
    left_camera_info_manager_ = std::make_shared<CameraInfoManager>(
        left_camera_node, left_camera_name_, left_camera_info_url_);
    right_camera_info_manager_ = std::make_shared<CameraInfoManager>(
        right_camera_node, right_camera_name_, right_camera_info_url_);
        
    // Set image publishers
    left_img_pub_ = create_image_publisher(private_node_, left_image_topic_, 1);
    right_img_pub_ = create_image_publisher(private_node_, right_image_topic_, 1);
    
    // Set rectified image publishers
    left_rect_img_pub_ = create_image_publisher(private_node_, left_image_topic_+"_rect", 1);
    right_rect_img_pub_ = create_image_publisher(private_node_, right_image_topic_+"_rect", 1);
    
    // Set camera info publishers
    left_info_pub_ = create_publisher<CameraInfo>(private_node_, "left/camera_info", 1);
    right_info_pub_ = create_publisher<CameraInfo>(private_node_, "right/camera_info", 1); 
    
    // Initialize camera
    camera_.reset(new StereoCamera());
    camera_->setStereoSplitRatio(split_ratio_);
    
    // Open camera
    logInfo("Opening camera, parameters: width=" + std::to_string(width_) + 
            ", height=" + std::to_string(height_) + 
            ", fps=" + std::to_string(fps_) + 
            ", mjpeg=" + (use_mjpeg_ ? "true" : "false") + 
            ", device_index=" + std::to_string(device_index_));
    
    if (!camera_->openWithParams(width_, height_, fps_, use_mjpeg_, device_index_)) {
        logError("Cannot open camera!");
        shutdown();
        return;
    }
    
    logInfo("Camera successfully opened");
    camera_->printDeviceInfo();
    fps_ = camera_->getFps();
    
    // Set frame callback function
    camera_->setFrameCallback(
        std::bind(&StereoCameraNode::frameCallback, this, 
                  std::placeholders::_1, std::placeholders::_2, 
                  std::placeholders::_3, std::placeholders::_4),
        this);
    
    // Create timer for checking camera status
    auto timer_callback = std::bind(&StereoCameraNode::timerCallback, this);
    timer_ = create_wall_timer(node_, Duration(1.0, 0.0), timer_callback);
    
    logInfo("Stereo camera node initialized");
}

StereoCameraNode::~StereoCameraNode() {
    // Stop processing thread
    running_ = false;
    frame_condition_.notify_all();
    
    // Wait for processing thread to end
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // Close camera
    if (camera_) {
        camera_->close();
    }
    
    logInfo("Stereo camera node closed. Statistics: Processed " + std::to_string(processed_frames_.load()) + 
            " frames, Dropped " + std::to_string(dropped_frames_.load()) + " frames");
}

void StereoCameraNode::timerCallback() {
    if (camera_ && camera_->isOpened()) {
        logDebug("Camera running, FPS: " + std::to_string(current_fps_));
    } else {
        logWarn("Camera not open!");
    }
}

void StereoCameraNode::logInfo(const std::string& msg) {
    log_info(node_, msg);
}

void StereoCameraNode::logWarn(const std::string& msg) {
    log_warn(node_, msg);
}

void StereoCameraNode::logError(const std::string& msg) {
    log_error(node_, msg);
}

void StereoCameraNode::logDebug(const std::string& msg) {
    log_debug(node_, msg);
}

#ifdef USE_ROS2
int main(int argc, char** argv) {
    init(argc, argv, "stereo_camera_node");
    auto node = create_node("stereo_camera_node");
    // In ROS2, private node naming can't use ~, we use the correct naming method
    auto private_node = create_node("stereo_camera_node_private");
    
    auto camera_node = std::make_shared<StereoCameraNode>(node, private_node);
    
    ros_spin(node);  // Use ros_spin instead of spin to avoid conflicts
    
    return 0;
}
#else
int main(int argc, char** argv) {
    init(argc, argv, "stereo_camera_node");
    auto node = create_node("");
    auto private_node = create_node("~");
    
    StereoCameraNode camera_node(node, private_node);
    
    ros_spin();  // Use ros_spin instead of spin to avoid conflicts
    
    return 0;
}
#endif
