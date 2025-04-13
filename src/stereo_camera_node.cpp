#include "stereo_camera_ros/stereo_camera_node.h"

StereoCameraNode::StereoCameraNode(NodePtr& main_node, NodePtr& private_node)
    : node_(main_node), private_node_(private_node),
      frame_buffer_(10), // Circular buffer with capacity for 10 frames
      running_(false), processing_thread_active_(false),
      dropped_frames_(0), processed_frames_(0) {
    
    // 加载参数
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
    
    // 获取包路径用于标定文件
    std::string pkg_path = get_package_share_directory("stereo_camera_ros");
    std::string left_calib_path = "file://" + pkg_path + "/config/left_camera.yaml";
    std::string right_calib_path = "file://" + pkg_path + "/config/right_camera.yaml";
    
    // 如果未指定相机信息URL，则使用默认值
    if (left_camera_info_url_.empty()) {
        left_camera_info_url_ = left_calib_path;
        logInfo("使用左相机标定文件: " + left_camera_info_url_);
    }
    
    if (right_camera_info_url_.empty()) {
        right_camera_info_url_ = right_calib_path;
        logInfo("使用右相机标定文件: " + right_camera_info_url_);
    }
    
    // 初始化相机信息管理器
    auto left_camera_node = get_camera_info_node(node_);
    auto right_camera_node = get_camera_info_node(node_);
    
    left_camera_info_manager_ = std::make_shared<CameraInfoManager>(
        left_camera_node, left_camera_name_, left_camera_info_url_);
    right_camera_info_manager_ = std::make_shared<CameraInfoManager>(
        right_camera_node, right_camera_name_, right_camera_info_url_);
        
    // 设置图像发布者
    left_img_pub_ = create_image_publisher(node_, left_image_topic_, 1);
    right_img_pub_ = create_image_publisher(node_, right_image_topic_, 1);
    
    // 设置相机信息发布者
    left_info_pub_ = create_publisher<CameraInfo>(node_, "camera_info/left", 1);
    right_info_pub_ = create_publisher<CameraInfo>(node_, "camera_info/right", 1);
    
    // 初始化相机
    camera_.reset(new StereoCamera());
    camera_->setStereoSplitRatio(split_ratio_);
    
    // 打开相机
    logInfo("打开相机，参数：width=" + std::to_string(width_) + 
            ", height=" + std::to_string(height_) + 
            ", fps=" + std::to_string(fps_) + 
            ", mjpeg=" + (use_mjpeg_ ? "true" : "false") + 
            ", device_index=" + std::to_string(device_index_));
    
    if (!camera_->openWithParams(width_, height_, fps_, use_mjpeg_, device_index_)) {
        logError("无法打开相机！");
        shutdown();
        return;
    }
    
    logInfo("相机已成功打开");
    camera_->printDeviceInfo();
    fps_ = camera_->getFps();
    
    // 设置帧回调函数
    camera_->setFrameCallback(
        std::bind(&StereoCameraNode::frameCallback, this, 
                  std::placeholders::_1, std::placeholders::_2, 
                  std::placeholders::_3, std::placeholders::_4),
        this);
    
    // 创建用于检查相机状态的定时器
    auto timer_callback = std::bind(&StereoCameraNode::timerCallback, this);
    timer_ = create_wall_timer(node_, Duration(1.0, 0.0), timer_callback);
    
    logInfo("双目相机节点已初始化");
}

StereoCameraNode::~StereoCameraNode() {
    // 停止处理线程
    running_ = false;
    frame_condition_.notify_all();
    
    // 等待处理线程结束
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // 关闭相机
    if (camera_) {
        camera_->close();
    }
    
    logInfo("双目相机节点已关闭. 统计: 已处理 " + std::to_string(processed_frames_.load()) + 
            " 帧, 丢弃 " + std::to_string(dropped_frames_.load()) + " 帧");
}

void StereoCameraNode::timerCallback() {
    if (camera_ && camera_->isOpened()) {
        logDebug("相机运行中，FPS: " + std::to_string(current_fps_));
    } else {
        logWarn("相机未打开！");
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
    // 在ROS2中，private节点命名不能用~，我们使用正确的命名方式
    auto private_node = create_node("stereo_camera_node_private");
    
    auto camera_node = std::make_shared<StereoCameraNode>(node, private_node);
    
    ros_spin(node);  // 使用ros_spin替代spin避免冲突
    
    return 0;
}
#else
int main(int argc, char** argv) {
    init(argc, argv, "stereo_camera_node");
    auto node = create_node("");
    auto private_node = create_node("~");
    
    StereoCameraNode camera_node(node, private_node);
    
    ros_spin();  // 使用ros_spin替代spin避免冲突
    
    return 0;
}
#endif
