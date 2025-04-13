#include "stereo_camera.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

StereoCamera::StereoCamera() 
    : m_width(1920), m_height(1080), m_fps(30), m_use_mjpeg(true), 
      m_split_ratio(0.5), m_ctx(nullptr), m_dev(nullptr), m_devh(nullptr),
      m_callback(nullptr), m_user_data(nullptr), m_running(false),
      m_is_opened(false), m_new_frame_available(false), m_latest_timestamp(0),
      m_workers_running(false), m_drop_threshold(10), m_dropped_frames(0)
{
}

StereoCamera::~StereoCamera() {
    close();
}

bool StereoCamera::open(int device_index) {
    return openWithParams(m_width, m_height, m_fps, m_use_mjpeg, device_index);
}

bool StereoCamera::openWithParams(int width, int height, int fps, bool use_mjpeg, int device_index, int worker_threads) {
    // 防止重复打开
    if (m_is_opened) {
        std::cerr << "相机已经打开，请先关闭" << std::endl;
        return false;
    }
    
    // 保存参数
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_use_mjpeg = use_mjpeg;
    
    // 初始化UVC
    if (!initializeUVC(device_index)) {
        cleanupUVC();
        return false;
    }
    
    // 检查相机格式和参数
    if (!checkCameraFormat()) {
        cleanupUVC();
        return false;
    }

    // 启动工作线程
    startWorkerThreads(worker_threads);
    
    m_is_opened = true;
    return true;
}

void StereoCamera::close() {
    if (!m_is_opened) return;
    
    m_running = false;
    
    // 停止工作线程
    stopWorkerThreads();
    
    // 清理UVC资源
    cleanupUVC();
    
    // 清理帧缓存
    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        m_latest_left.release();
        m_latest_right.release();
        m_new_frame_available = false;
    }
    
    // 清理队列
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);
        while (!m_frame_queue.empty()) {
            auto& data = m_frame_queue.front();
            uvc_free_frame(data.frame);
            m_frame_queue.pop();
        }
    }
    
    // 清理转换缓冲区
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_conversion_buffers.clear();
    }
    
    m_is_opened = false;
    std::cout << "相机已关闭" << std::endl;
}

bool StereoCamera::isOpened() const {
    return m_is_opened;
}

void StereoCamera::setFrameCallback(FrameCallback callback, void* user_data) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callback = callback;
    m_user_data = user_data;
}

void StereoCamera::setStereoSplitRatio(float split_ratio) {
    if (split_ratio < 0.1f || split_ratio > 0.9f) {
        std::cerr << "无效的分割比例，必须在0.1-0.9之间" << std::endl;
        return;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_split_ratio = split_ratio;
}

void StereoCamera::setFrameDropThreshold(size_t threshold) {
    std::lock_guard<std::mutex> lock(m_queue_mutex);
    m_drop_threshold = threshold;
}

size_t StereoCamera::getQueueLength() const {
    std::lock_guard<std::mutex> lock(m_queue_mutex);
    return m_frame_queue.size();
}

void StereoCamera::printDeviceInfo() const {
    if (!m_devh) {
        std::cerr << "相机未打开" << std::endl;
        return;
    }
    
    uvc_device_descriptor_t* desc;
    uvc_get_device_descriptor(m_dev, &desc);
    
    std::cout << "设备信息:" << std::endl;
    std::cout << "  - 制造商: " << (desc->manufacturer ? desc->manufacturer : "未知") << std::endl;
    std::cout << "  - 产品名: " << (desc->product ? desc->product : "未知") << std::endl;
    std::cout << "  - 序列号: " << (desc->serialNumber ? desc->serialNumber : "未知") << std::endl;
    std::cout << "  - 当前分辨率: " << m_width << "x" << m_height << std::endl;
    std::cout << "  - 当前帧率: " << m_fps << " fps" << std::endl;
    std::cout << "  - 格式: " << (m_use_mjpeg ? "MJPEG" : "YUYV") << std::endl;
    
    uvc_free_device_descriptor(desc);
    
    // 打印支持的格式
    const uvc_format_desc_t* format_desc = uvc_get_format_descs(m_devh);
    std::cout << "支持的格式:" << std::endl;
    
    while (format_desc) {
        std::string format_name;
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG) {
            format_name = "MJPEG";
        } else if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED) {
            format_name = "YUYV";
        } else {
            format_name = "其他格式";
        }
        
        std::cout << "  - " << format_name << ":" << std::endl;
        
        const uvc_frame_desc_t* frame_desc = format_desc->frame_descs;
        while (frame_desc) {
            int max_fps = 0;
            if (frame_desc->intervals) {
                max_fps = 10000000 / frame_desc->intervals[0];
            }
            
            std::cout << "    " << frame_desc->wWidth << "x" << frame_desc->wHeight
                      << " @ " << max_fps << " fps" << std::endl;
            
            frame_desc = frame_desc->next;
        }
        
        format_desc = format_desc->next;
    }
}

int StereoCamera::getWidth() const { return m_width; }
int StereoCamera::getHeight() const { return m_height; }
int StereoCamera::getFps() const { return m_fps; }
bool StereoCamera::isMjpegMode() const { return m_use_mjpeg; }

bool StereoCamera::getLatestFrames(cv::Mat& left, cv::Mat& right, uint64_t& timestamp, 
                                  int timeout_ms) {
    if (!m_is_opened) {
        std::cerr << "相机未打开" << std::endl;
        return false;
    }
    
    std::unique_lock<std::mutex> lock(m_frame_mutex);
    
    if (timeout_ms > 0) {
        // 等待条件变量通知有新帧可用，或者超时
        bool result = m_frame_cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                           [this]() { return m_new_frame_available; });
        if (!result) {
            // 超时
            return false;
        }
    } else if (!m_new_frame_available) {
        // 如果没有超时且没有新帧，直接返回失败
        return false;
    }
    
    // 复制最新帧
    m_latest_left.copyTo(left);
    m_latest_right.copyTo(right);
    timestamp = m_latest_timestamp;
    
    return true;
}

void StereoCamera::uvcFrameCallback(uvc_frame_t* frame, void* ptr) {
    // 静态回调，转发到类的成员函数
    StereoCamera* camera = static_cast<StereoCamera*>(ptr);
    if (camera) {
        camera->processFrame(frame);
    }
}

void StereoCamera::processFrame(uvc_frame_t* frame) {
    if (!frame || frame->data_bytes == 0) return;
    
    // 获取当前时间戳（纳秒）
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    // 记录帧时间戳历史，用于计算实际帧率
    m_frame_timestamp_history.push_back(timestamp);
    while(m_frame_timestamp_history.size() > 100) {
        m_frame_timestamp_history.pop_front();
    }

    // 打印帧率信息（每秒一次）
    // static uint64_t last_show_fps_time = 0;
    // if(m_frame_timestamp_history.size() > 2 && timestamp - last_show_fps_time > 1e9) {
    //     last_show_fps_time = timestamp;
    //     double actual_duration = (m_frame_timestamp_history.back() - m_frame_timestamp_history.front()) * 1e-9;
    //     double actual_fps = (m_frame_timestamp_history.size() - 1) / actual_duration;
        
    //     size_t queue_size = getQueueLength();
    //     size_t dropped = m_dropped_frames.load();
        
    //     std::cout << "接收帧率: " << std::fixed << std::setprecision(1) << actual_fps
    //         << " fps, 队列长度=" << queue_size
    //         << ", 丢弃帧数=" << dropped
    //         << ", 分辨率=" << frame->width << "x" << frame->height
    //         << ", 设定帧率=" << getFps() << std::endl;
    // }
    
    // auto t0 = std::chrono::steady_clock::now();
    // 复制一份帧数据，因为UVC回调函数返回后可能会释放原始帧
    uvc_frame_t* frame_copy = uvc_allocate_frame(frame->data_bytes);
    if (!frame_copy) {
        std::cerr << "无法分配内存复制帧" << std::endl;
        return;
    }
    // auto t1 = std::chrono::steady_clock::now();
    
    uvc_error_t ret = uvc_duplicate_frame(frame, frame_copy);
    if (ret < 0) {
        std::cerr << "复制帧失败: " << uvc_strerror(ret) << std::endl;
        uvc_free_frame(frame_copy);
        return;
    }
    // auto t2 =  std::chrono::steady_clock::now();
    // std::cout << "开辟内存耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us， "
    //           << "复制帧耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
    
    // 将帧放入处理队列
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);
        
        // 如果队列过长，丢弃旧帧
        if (m_drop_threshold > 0 && m_frame_queue.size() >= m_drop_threshold) {
            // 丢弃最老的一帧
            auto& old_frame = m_frame_queue.front();
            uvc_free_frame(old_frame.frame);
            m_frame_queue.pop();
            m_dropped_frames++;
        }
        
        // 添加新帧到队列
        FrameData data = {frame_copy, timestamp};
        m_frame_queue.push(data);
    }
    
    // 通知工作线程处理新帧
    m_queue_cond.notify_one();
}

void StereoCamera::workerThread() {
    while (m_workers_running) {
        // 从队列中获取一帧数据
        FrameData data;
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            // 等待队列中有数据或者停止信号
            m_queue_cond.wait(lock, [this] {
                return !m_frame_queue.empty() || !m_workers_running;
            });
            
            // 检查退出条件
            if (!m_workers_running) break;
            if (m_frame_queue.empty()) continue;
            
            // 获取并移除队列头部的帧
            data = m_frame_queue.front();
            m_frame_queue.pop();
        }
        
        auto t0 = std::chrono::steady_clock::now();
        // 处理帧
        uvc_frame_t* frame = data.frame;
        uint64_t timestamp = data.timestamp;
        
        // 获取或分配转换缓冲区
        cv::Mat conversion_buffer;
        {
            std::lock_guard<std::mutex> lock(m_buffer_mutex);
            if (!m_conversion_buffers.empty()) {
                conversion_buffer = m_conversion_buffers.back();
                m_conversion_buffers.pop_back();
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        
        // 转换为OpenCV格式
        cv::Mat full_frame;
        if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
            full_frame = mjpegToBgr(static_cast<uint8_t*>(frame->data), frame->data_bytes, conversion_buffer);
        } 
        else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
            full_frame = yuyvToBgr(static_cast<uint8_t*>(frame->data), frame->width, frame->height);
        }
        else {
            std::cerr << "不支持的格式: " << frame->frame_format << std::endl;
            uvc_free_frame(frame);
            return;
        }
        auto t2 = std::chrono::steady_clock::now();
        
        // 释放复制的帧
        uvc_free_frame(frame);
        auto t3 = std::chrono::steady_clock::now();
        
        // 归还转换缓冲区
        if (!conversion_buffer.empty()) {
            std::lock_guard<std::mutex> lock(m_buffer_mutex);
            m_conversion_buffers.push_back(conversion_buffer);
        }
        
        if (full_frame.empty()) {
            std::cerr << "帧转换失败" << std::endl;
            continue;
        }
        
        // 分割为左右眼图像
        cv::Mat left_frame, right_frame;
        splitStereoFrame(full_frame, left_frame, right_frame);
        auto t4 = std::chrono::steady_clock::now();
        
        // 更新最新帧缓存
        {
            std::lock_guard<std::mutex> lock(m_frame_mutex);
            // left_frame.copyTo(m_latest_left);
            // right_frame.copyTo(m_latest_right);
            m_latest_left = left_frame;
            m_latest_right = right_frame;
            m_latest_timestamp = timestamp;
            m_new_frame_available = true;
        }
        
        // 通知等待的线程
        m_frame_cond.notify_all();
        
        // 调用用户回调
        FrameCallback callback;
        void* user_data;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            callback = m_callback;
            user_data = m_user_data;
        }
        auto t5 = std::chrono::steady_clock::now();
        
        if (callback) {
            try {
                callback(left_frame, right_frame, timestamp, user_data);
            } catch (const std::exception& e) {
                std::cerr << "回调函数异常: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "回调函数未知异常" << std::endl;
            }
        }
        auto t6 = std::chrono::steady_clock::now();
        // std::cout << "处理帧耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us, "
        //           << "转换耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us, "
        //           << "释放帧耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << "us, "
        //           << "分割耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << "us, "
        //           << "回调耗时: " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() << "us, " 
        //           << "总耗时： " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t0).count() << "us"
        //           << std::endl;
    }
}

void StereoCamera::startWorkerThreads(int thread_count) {
    stopWorkerThreads();  // 确保之前的线程已经停止
    
    // 预分配转换缓冲区
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_conversion_buffers.resize(thread_count);
    }
    
    m_dropped_frames = 0;
    m_workers_running = true;
    
    // 创建工作线程
    for (int i = 0; i < thread_count; ++i) {
        m_worker_threads.emplace_back(&StereoCamera::workerThread, this);
    }
    
    std::cout << "启动 " << thread_count << " 个工作线程" << std::endl;
}

void StereoCamera::stopWorkerThreads() {
    if (!m_workers_running) return;
    
    // 停止工作线程
    m_workers_running = false;
    m_queue_cond.notify_all();
    
    // 等待所有线程结束
    for (auto& thread : m_worker_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    m_worker_threads.clear();
    std::cout << "已停止所有工作线程" << std::endl;
}

void StereoCamera::splitStereoFrame(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right) const {
    if (stereo_frame.empty()) return;
    
    int split_point = static_cast<int>(stereo_frame.cols * m_split_ratio);
    
    // 创建ROI分别对应左右两个相机
    cv::Rect left_roi(0, 0, split_point, stereo_frame.rows);
    cv::Rect right_roi(split_point, 0, stereo_frame.cols - split_point, stereo_frame.rows);
    
    // 将ROI区域复制到输出图像
    // stereo_frame(left_roi).copyTo(left);
    // stereo_frame(right_roi).copyTo(right);
    left = stereo_frame(left_roi);
    right = stereo_frame(right_roi);
}

cv::Mat StereoCamera::mjpegToBgr(const uint8_t* mjpeg_data, size_t mjpeg_size, cv::Mat& buffer) const {
    try {
        // 解码JPEG数据
        cv::Mat jpeg_data(1, mjpeg_size, CV_8UC1, const_cast<uint8_t*>(mjpeg_data));

        // 使用预分配的缓冲区（如果有）
        if (!buffer.empty()) {
            // 尝试直接解码到预分配缓冲区
            cv::imdecode(jpeg_data, cv::IMREAD_COLOR, &buffer);
            return buffer;
        }

        // 否则创建新的Mat
        return cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV错误: " << e.what() << std::endl;
        return cv::Mat();
    }
}

cv::Mat StereoCamera::yuyvToBgr(const uint8_t* yuyv_data, int width, int height) const {
    try {
        // 创建OpenCV Mat
        cv::Mat yuyv(height, width, CV_8UC2, const_cast<uint8_t*>(yuyv_data));
        cv::Mat bgr;
        // YUYV到BGR的转换
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return bgr;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV错误: " << e.what() << std::endl;
        return cv::Mat();
    }
}

bool StereoCamera::checkCameraFormat() {
    if (!m_devh) return false;
    
    // 获取相机支持格式列表
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(m_devh);
    
    // 首先尝试MJPEG
    bool mjpeg_supported = false;
    bool yuyv_supported = false;
    uint32_t max_mjpeg_width = 0, max_mjpeg_height = 0;
    uint32_t max_yuyv_width = 0, max_yuyv_height = 0;
    int max_mjpeg_fps = 0;
    int max_yuyv_fps = 0;
    
    while (format_desc) {
        // 检查是否为MJPEG格式
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG) {
            mjpeg_supported = true;
            
            // 检查支持的分辨率
            uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
            while (frame_desc) {
                if (frame_desc->wWidth >= max_mjpeg_width && frame_desc->wHeight >= max_mjpeg_height) {
                    max_mjpeg_width = frame_desc->wWidth;
                    max_mjpeg_height = frame_desc->wHeight;
                    
                    // 计算最大帧率
                    if (frame_desc->intervals) {
                        max_mjpeg_fps = 10000000 / frame_desc->intervals[0];
                    }
                }
                frame_desc = frame_desc->next;
            }
        }
        
        // 检查是否为YUYV格式
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED && 
            format_desc->guidFormat[0] == 'Y' && format_desc->guidFormat[1] == 'U' && 
            format_desc->guidFormat[2] == 'Y' && format_desc->guidFormat[3] == '2') {
            yuyv_supported = true;
            
            // 检查支持的分辨率
            uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
            while (frame_desc) {
                if (frame_desc->wWidth >= max_yuyv_width && frame_desc->wHeight >= max_yuyv_height) {
                    max_yuyv_width = frame_desc->wWidth;
                    max_yuyv_height = frame_desc->wHeight;
                    
                    // 计算最大帧率
                    if (frame_desc->intervals) {
                        max_yuyv_fps = 10000000 / frame_desc->intervals[0];
                    }
                }
                frame_desc = frame_desc->next;
            }
        }
        
        format_desc = format_desc->next;
    }
    
    // 输出支持信息
    std::cout << "摄像头支持信息：" << std::endl;
    if (mjpeg_supported) {
        std::cout << "支持MJPEG格式，最大分辨率: " << max_mjpeg_width << "x" 
                  << max_mjpeg_height << " @ " << max_mjpeg_fps << "fps" << std::endl;
    }
    if (yuyv_supported) {
        std::cout << "支持YUYV格式，最大分辨率: " << max_yuyv_width << "x" 
                  << max_yuyv_height << " @ " << max_yuyv_fps << "fps" << std::endl;
    }
    
    // 选择最佳格式
    bool format_ok = false;
    uint32_t actual_width = m_width;
    uint32_t actual_height = m_height;
    int actual_fps = m_fps;
    bool actual_mjpeg = m_use_mjpeg;
    
    if (m_use_mjpeg && mjpeg_supported && max_mjpeg_width >= m_width && max_mjpeg_height >= m_height) {
        actual_width = std::min(static_cast<uint32_t>(m_width), max_mjpeg_width);
        actual_height = std::min(static_cast<uint32_t>(m_height), max_mjpeg_height);
        actual_fps = std::min(m_fps, max_mjpeg_fps);
        actual_mjpeg = true;
        format_ok = true;
        std::cout << "选择MJPEG格式: " << actual_width << "x" << actual_height 
                  << " @ " << actual_fps << "fps" << std::endl;
    } else if (yuyv_supported && max_yuyv_width >= m_width && max_yuyv_height >= m_height) {
        actual_width = std::min(static_cast<uint32_t>(m_width), max_yuyv_width);
        actual_height = std::min(static_cast<uint32_t>(m_height), max_yuyv_height);
        actual_fps = std::min(m_fps, max_yuyv_fps);
        actual_mjpeg = false;
        format_ok = true;
        std::cout << "选择YUYV格式: " << actual_width << "x" << actual_height 
                  << " @ " << actual_fps << "fps" << std::endl;
    }
    
    if (!format_ok) {
        // 尝试降级到支持的格式
        if (mjpeg_supported) {
            actual_width = max_mjpeg_width;
            actual_height = max_mjpeg_height;
            actual_fps = max_mjpeg_fps;
            actual_mjpeg = true;
            format_ok = true;
            std::cout << "降级到MJPEG格式: " << actual_width << "x" << actual_height 
                      << " @ " << actual_fps << "fps" << std::endl;
        } else if (yuyv_supported) {
            actual_width = max_yuyv_width;
            actual_height = max_yuyv_height;
            actual_fps = max_yuyv_fps;
            actual_mjpeg = false;
            format_ok = true;
            std::cout << "降级到YUYV格式: " << actual_width << "x" << actual_height 
                      << " @ " << actual_fps << "fps" << std::endl;
        }
    }
    
    if (!format_ok) {
        std::cerr << "错误: 相机不支持请求的格式和分辨率" << std::endl;
        return false;
    }
    
    // 更新实际使用的参数
    m_width = actual_width;
    m_height = actual_height;
    m_fps = actual_fps;
    m_use_mjpeg = actual_mjpeg;
    
    // 获取流控制
    uvc_error_t res;
    if (m_use_mjpeg) {
        res = uvc_get_stream_ctrl_format_size(
            m_devh, &m_ctrl,
            UVC_FRAME_FORMAT_MJPEG,
            m_width, m_height, m_fps
        );
    } else {
        res = uvc_get_stream_ctrl_format_size(
            m_devh, &m_ctrl,
            UVC_FRAME_FORMAT_YUYV,
            m_width, m_height, m_fps
        );
    }
    
    if (res < 0) {
        std::cerr << "获取流控制失败: " << uvc_strerror(res) << std::endl;
        return false;
    }
    
    // 打印流控制信息
    uvc_print_stream_ctrl(&m_ctrl, stderr);
    
    // 开始流
    m_running = true;
    res = uvc_start_streaming(m_devh, &m_ctrl, uvcFrameCallback, this, 0);
    
    if (res < 0) {
        std::cerr << "开始流失败: " << uvc_strerror(res) << std::endl;
        m_running = false;
        return false;
    }
    
    return true;
}

bool StereoCamera::initializeUVC(int device_index) {
    uvc_error_t res;
    
    // 初始化UVC上下文
    res = uvc_init(&m_ctx, NULL);
    if (res < 0) {
        std::cerr << "UVC初始化失败: " << uvc_strerror(res) << std::endl;
        return false;
    }
    
    // 获取设备列表
    uvc_device_t **device_list;
    res = uvc_get_device_list(m_ctx, &device_list);
    if (res < 0) {
        std::cerr << "无法获取设备列表: " << uvc_strerror(res) << std::endl;
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    // 检查设备索引是否有效
    int device_count = 0;
    while (device_list[device_count] != nullptr) {
        device_count++;
    }
    
    if (device_index >= device_count) {
        std::cerr << "无效的设备索引: " << device_index << ", 只有 " << device_count << " 个设备" << std::endl;
        uvc_free_device_list(device_list, 1);
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    // 获取指定索引的设备
    m_dev = device_list[device_index];
    uvc_ref_device(m_dev);
    uvc_free_device_list(device_list, 1);
    
    // 打开设备
    res = uvc_open(m_dev, &m_devh);
    if (res < 0) {
        std::cerr << "无法打开设备: " << uvc_strerror(res) << std::endl;
        uvc_unref_device(m_dev);
        m_dev = nullptr;
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    return true;
}

void StereoCamera::cleanupUVC() {
    // 停止流
    if (m_running) {
        uvc_stop_streaming(m_devh);
        m_running = false;
    }
    
    // 关闭设备
    if (m_devh) {
        uvc_close(m_devh);
        m_devh = nullptr;
    }
    
    // 清理设备引用
    if (m_dev) {
        uvc_unref_device(m_dev);
        m_dev = nullptr;
    }
    
    // 退出UVC上下文
    if (m_ctx) {
        uvc_exit(m_ctx);
        m_ctx = nullptr;
    }
}