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
    // Prevent reopening
    if (m_is_opened) {
        std::cerr << "Camera is already open, please close it first" << std::endl;
        return false;
    }
    
    // Save parameters
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_use_mjpeg = use_mjpeg;
    
    // Initialize UVC
    if (!initializeUVC(device_index)) {
        cleanupUVC();
        return false;
    }
    
    // Check camera format and parameters
    if (!checkCameraFormat()) {
        cleanupUVC();
        return false;
    }

    // Start worker threads
    startWorkerThreads(worker_threads);
    
    m_is_opened = true;
    return true;
}

void StereoCamera::close() {
    if (!m_is_opened) return;
    
    m_running = false;
    
    // Stop worker threads
    stopWorkerThreads();
    
    // Clean up UVC resources
    cleanupUVC();
    
    // Clean frame cache
    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        m_latest_left.release();
        m_latest_right.release();
        m_new_frame_available = false;
    }
    
    // Clean queue
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);
        while (!m_frame_queue.empty()) {
            auto& data = m_frame_queue.front();
            uvc_free_frame(data.frame);
            m_frame_queue.pop();
        }
    }
    
    // Clean conversion buffers
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_conversion_buffers.clear();
    }
    
    m_is_opened = false;
    std::cout << "Camera closed" << std::endl;
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
        std::cerr << "Invalid split ratio, must be between 0.1-0.9" << std::endl;
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
        std::cerr << "Camera not open" << std::endl;
        return;
    }
    
    uvc_device_descriptor_t* desc;
    uvc_get_device_descriptor(m_dev, &desc);
    
    std::cout << "Device information:" << std::endl;
    std::cout << "  - Manufacturer: " << (desc->manufacturer ? desc->manufacturer : "Unknown") << std::endl;
    std::cout << "  - Product name: " << (desc->product ? desc->product : "Unknown") << std::endl;
    std::cout << "  - Serial number: " << (desc->serialNumber ? desc->serialNumber : "Unknown") << std::endl;
    std::cout << "  - Current resolution: " << m_width << "x" << m_height << std::endl;
    std::cout << "  - Current frame rate: " << m_fps << " fps" << std::endl;
    std::cout << "  - Format: " << (m_use_mjpeg ? "MJPEG" : "YUYV") << std::endl;
    
    uvc_free_device_descriptor(desc);
    
    // Print supported formats
    const uvc_format_desc_t* format_desc = uvc_get_format_descs(m_devh);
    std::cout << "Supported formats:" << std::endl;
    
    while (format_desc) {
        std::string format_name;
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG) {
            format_name = "MJPEG";
        } else if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED) {
            format_name = "YUYV";
        } else {
            format_name = "Other format";
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
        std::cerr << "Camera not open" << std::endl;
        return false;
    }
    
    std::unique_lock<std::mutex> lock(m_frame_mutex);
    
    if (timeout_ms > 0) {
        // Wait for condition variable to notify new frame available, or timeout
        bool result = m_frame_cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                           [this]() { return m_new_frame_available; });
        if (!result) {
            // Timeout
            return false;
        }
    } else if (!m_new_frame_available) {
        // If no timeout and no new frame, return failure immediately
        return false;
    }
    
    // Copy the latest frame
    m_latest_left.copyTo(left);
    m_latest_right.copyTo(right);
    timestamp = m_latest_timestamp;
    
    return true;
}

void StereoCamera::uvcFrameCallback(uvc_frame_t* frame, void* ptr) {
    // Static callback, forward to class member function
    StereoCamera* camera = static_cast<StereoCamera*>(ptr);
    if (camera) {
        camera->processFrame(frame);
    }
}

void StereoCamera::processFrame(uvc_frame_t* frame) {
    if (!frame || frame->data_bytes == 0) return;
    
    // Get current timestamp (nanoseconds)
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    // Record frame timestamp history for calculating actual frame rate
    m_frame_timestamp_history.push_back(timestamp);
    while(m_frame_timestamp_history.size() > 100) {
        m_frame_timestamp_history.pop_front();
    }

    // Print frame rate info (once per second)
    // static uint64_t last_show_fps_time = 0;
    // if(m_frame_timestamp_history.size() > 2 && timestamp - last_show_fps_time > 1e9) {
    //     last_show_fps_time = timestamp;
    //     double actual_duration = (m_frame_timestamp_history.back() - m_frame_timestamp_history.front()) * 1e-9;
    //     double actual_fps = (m_frame_timestamp_history.size() - 1) / actual_duration;
        
    //     size_t queue_size = getQueueLength();
    //     size_t dropped = m_dropped_frames.load();
        
    //     std::cout << "Received frame rate: " << std::fixed << std::setprecision(1) << actual_fps
    //         << " fps, Queue length=" << queue_size
    //         << ", Dropped frames=" << dropped
    //         << ", Resolution=" << frame->width << "x" << frame->height
    //         << ", Set frame rate=" << getFps() << std::endl;
    // }
    
    // auto t0 = std::chrono::steady_clock::now();
    // Copy the frame data, since UVC callback function may release the original frame after return
    uvc_frame_t* frame_copy = uvc_allocate_frame(frame->data_bytes);
    if (!frame_copy) {
        std::cerr << "Cannot allocate memory to copy frame" << std::endl;
        return;
    }
    // auto t1 = std::chrono::steady_clock::now();
    
    uvc_error_t ret = uvc_duplicate_frame(frame, frame_copy);
    if (ret < 0) {
        std::cerr << "Failed to copy frame: " << uvc_strerror(ret) << std::endl;
        uvc_free_frame(frame_copy);
        return;
    }
    // auto t2 =  std::chrono::steady_clock::now();
    // std::cout << "Memory allocation time: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us, "
    //           << "Frame copy time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
    
    // Put frame in processing queue
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);
        
        // If queue is too long, drop old frames
        if (m_drop_threshold > 0 && m_frame_queue.size() >= m_drop_threshold) {
            // Drop the oldest frame
            auto& old_frame = m_frame_queue.front();
            uvc_free_frame(old_frame.frame);
            m_frame_queue.pop();
            m_dropped_frames++;
        }
        
        // Add new frame to queue
        FrameData data = {frame_copy, timestamp};
        m_frame_queue.push(data);
    }
    
    // Notify worker threads to process new frame
    m_queue_cond.notify_one();
}

void StereoCamera::workerThread() {
    while (m_workers_running) {
        // Get one frame from queue
        FrameData data;
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            // Wait for data in queue or stop signal
            m_queue_cond.wait(lock, [this] {
                return !m_frame_queue.empty() || !m_workers_running;
            });
            
            // Check exit condition
            if (!m_workers_running) break;
            if (m_frame_queue.empty()) continue;
            
            // Get and remove frame from queue head
            data = m_frame_queue.front();
            m_frame_queue.pop();
        }
        
        auto t0 = std::chrono::steady_clock::now();
        // Process frame
        uvc_frame_t* frame = data.frame;
        uint64_t timestamp = data.timestamp;
        
        // Get or allocate conversion buffer
        cv::Mat conversion_buffer;
        {
            std::lock_guard<std::mutex> lock(m_buffer_mutex);
            if (!m_conversion_buffers.empty()) {
                conversion_buffer = m_conversion_buffers.back();
                m_conversion_buffers.pop_back();
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        
        // Convert to OpenCV format
        cv::Mat full_frame;
        if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
            full_frame = mjpegToBgr(static_cast<uint8_t*>(frame->data), frame->data_bytes, conversion_buffer);
        } 
        else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
            full_frame = yuyvToBgr(static_cast<uint8_t*>(frame->data), frame->width, frame->height);
        }
        else {
            std::cerr << "Unsupported format: " << frame->frame_format << std::endl;
            uvc_free_frame(frame);
            return;
        }
        auto t2 = std::chrono::steady_clock::now();
        
        // Release copied frame
        uvc_free_frame(frame);
        auto t3 = std::chrono::steady_clock::now();
        
        // Return conversion buffer
        if (!conversion_buffer.empty()) {
            std::lock_guard<std::mutex> lock(m_buffer_mutex);
            m_conversion_buffers.push_back(conversion_buffer);
        }
        
        if (full_frame.empty()) {
            std::cerr << "Frame conversion failed" << std::endl;
            continue;
        }
        
        // Split into left and right eye images
        cv::Mat left_frame, right_frame;
        splitStereoFrame(full_frame, left_frame, right_frame);
        auto t4 = std::chrono::steady_clock::now();
        
        // Update latest frame cache
        {
            std::lock_guard<std::mutex> lock(m_frame_mutex);
            // left_frame.copyTo(m_latest_left);
            // right_frame.copyTo(m_latest_right);
            m_latest_left = left_frame;
            m_latest_right = right_frame;
            m_latest_timestamp = timestamp;
            m_new_frame_available = true;
        }
        
        // Notify waiting threads
        m_frame_cond.notify_all();
        
        // Call user callback
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
                std::cerr << "Callback function exception: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Callback function unknown exception" << std::endl;
            }
        }
        auto t6 = std::chrono::steady_clock::now();
        // std::cout << "Frame processing time: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us, "
        //           << "Conversion time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us, "
        //           << "Frame release time: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << "us, "
        //           << "Split time: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << "us, "
        //           << "Callback time: " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() << "us, " 
        //           << "Total time: " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t0).count() << "us"
        //           << std::endl;
    }
}

void StereoCamera::startWorkerThreads(int thread_count) {
    stopWorkerThreads();  // Make sure previous threads are stopped
    
    // Pre-allocate conversion buffers
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_conversion_buffers.resize(thread_count);
    }
    
    m_dropped_frames = 0;
    m_workers_running = true;
    
    // Create worker threads
    for (int i = 0; i < thread_count; ++i) {
        m_worker_threads.emplace_back(&StereoCamera::workerThread, this);
    }
    
    std::cout << "Started " << thread_count << " worker threads" << std::endl;
}

void StereoCamera::stopWorkerThreads() {
    if (!m_workers_running) return;
    
    // Stop worker threads
    m_workers_running = false;
    m_queue_cond.notify_all();
    
    // Wait for all threads to end
    for (auto& thread : m_worker_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    m_worker_threads.clear();
    std::cout << "Stopped all worker threads" << std::endl;
}

void StereoCamera::splitStereoFrame(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right) const {
    if (stereo_frame.empty()) return;
    
    int split_point = static_cast<int>(stereo_frame.cols * m_split_ratio);
    
    // Create ROIs for left and right cameras
    cv::Rect left_roi(0, 0, split_point, stereo_frame.rows);
    cv::Rect right_roi(split_point, 0, stereo_frame.cols - split_point, stereo_frame.rows);
    
    // Copy ROI regions to output images
    // stereo_frame(left_roi).copyTo(left);
    // stereo_frame(right_roi).copyTo(right);
    left = stereo_frame(left_roi);
    right = stereo_frame(right_roi);
}

cv::Mat StereoCamera::mjpegToBgr(const uint8_t* mjpeg_data, size_t mjpeg_size, cv::Mat& buffer) const {
    try {
        // Decode JPEG data
        cv::Mat jpeg_data(1, mjpeg_size, CV_8UC1, const_cast<uint8_t*>(mjpeg_data));

        // Use pre-allocated buffer (if available)
        if (!buffer.empty()) {
            // Try to decode directly to pre-allocated buffer
            cv::imdecode(jpeg_data, cv::IMREAD_COLOR, &buffer);
            return buffer;
        }

        // Otherwise create new Mat
        return cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
        return cv::Mat();
    }
}

cv::Mat StereoCamera::yuyvToBgr(const uint8_t* yuyv_data, int width, int height) const {
    try {
        // Create OpenCV Mat
        cv::Mat yuyv(height, width, CV_8UC2, const_cast<uint8_t*>(yuyv_data));
        cv::Mat bgr;
        // YUYV to BGR conversion
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return bgr;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
        return cv::Mat();
    }
}

bool StereoCamera::checkCameraFormat() {
    if (!m_devh) return false;
    
    // Get camera supported format list
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(m_devh);
    
    // First try MJPEG
    bool mjpeg_supported = false;
    bool yuyv_supported = false;
    uint32_t max_mjpeg_width = 0, max_mjpeg_height = 0;
    uint32_t max_yuyv_width = 0, max_yuyv_height = 0;
    int max_mjpeg_fps = 0;
    int max_yuyv_fps = 0;
    
    while (format_desc) {
        // Check if MJPEG format
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG) {
            mjpeg_supported = true;
            
            // Check supported resolutions
            uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
            while (frame_desc) {
                if (frame_desc->wWidth >= max_mjpeg_width && frame_desc->wHeight >= max_mjpeg_height) {
                    max_mjpeg_width = frame_desc->wWidth;
                    max_mjpeg_height = frame_desc->wHeight;
                    
                    // Calculate max frame rate
                    if (frame_desc->intervals) {
                        max_mjpeg_fps = 10000000 / frame_desc->intervals[0];
                    }
                }
                frame_desc = frame_desc->next;
            }
        }
        
        // Check if YUYV format
        if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED && 
            format_desc->guidFormat[0] == 'Y' && format_desc->guidFormat[1] == 'U' && 
            format_desc->guidFormat[2] == 'Y' && format_desc->guidFormat[3] == '2') {
            yuyv_supported = true;
            
            // Check supported resolutions
            uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
            while (frame_desc) {
                if (frame_desc->wWidth >= max_yuyv_width && frame_desc->wHeight >= max_yuyv_height) {
                    max_yuyv_width = frame_desc->wWidth;
                    max_yuyv_height = frame_desc->wHeight;
                    
                    // Calculate max frame rate
                    if (frame_desc->intervals) {
                        max_yuyv_fps = 10000000 / frame_desc->intervals[0];
                    }
                }
                frame_desc = frame_desc->next;
            }
        }
        
        format_desc = format_desc->next;
    }
    
    // Output support information
    std::cout << "Camera support information:" << std::endl;
    if (mjpeg_supported) {
        std::cout << "Supports MJPEG format, max resolution: " << max_mjpeg_width << "x" 
                  << max_mjpeg_height << " @ " << max_mjpeg_fps << "fps" << std::endl;
    }
    if (yuyv_supported) {
        std::cout << "Supports YUYV format, max resolution: " << max_yuyv_width << "x" 
                  << max_yuyv_height << " @ " << max_yuyv_fps << "fps" << std::endl;
    }
    
    // Select best format
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
        std::cout << "Selected MJPEG format: " << actual_width << "x" << actual_height 
                  << " @ " << actual_fps << "fps" << std::endl;
    } else if (yuyv_supported && max_yuyv_width >= m_width && max_yuyv_height >= m_height) {
        actual_width = std::min(static_cast<uint32_t>(m_width), max_yuyv_width);
        actual_height = std::min(static_cast<uint32_t>(m_height), max_yuyv_height);
        actual_fps = std::min(m_fps, max_yuyv_fps);
        actual_mjpeg = false;
        format_ok = true;
        std::cout << "Selected YUYV format: " << actual_width << "x" << actual_height 
                  << " @ " << actual_fps << "fps" << std::endl;
    }
    
    if (!format_ok) {
        // Try falling back to supported formats
        if (mjpeg_supported) {
            actual_width = max_mjpeg_width;
            actual_height = max_mjpeg_height;
            actual_fps = max_mjpeg_fps;
            actual_mjpeg = true;
            format_ok = true;
            std::cout << "Falling back to MJPEG format: " << actual_width << "x" << actual_height 
                      << " @ " << actual_fps << "fps" << std::endl;
        } else if (yuyv_supported) {
            actual_width = max_yuyv_width;
            actual_height = max_yuyv_height;
            actual_fps = max_yuyv_fps;
            actual_mjpeg = false;
            format_ok = true;
            std::cout << "Falling back to YUYV format: " << actual_width << "x" << actual_height 
                      << " @ " << actual_fps << "fps" << std::endl;
        }
    }
    
    if (!format_ok) {
        std::cerr << "Error: Camera doesn't support requested format and resolution" << std::endl;
        return false;
    }
    
    // Update actual parameters
    m_width = actual_width;
    m_height = actual_height;
    m_fps = actual_fps;
    m_use_mjpeg = actual_mjpeg;
    
    // Get stream control
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
        std::cerr << "Get stream control failed: " << uvc_strerror(res) << std::endl;
        return false;
    }
    
    // Print stream control info
    uvc_print_stream_ctrl(&m_ctrl, stderr);
    
    // Start stream
    m_running = true;
    res = uvc_start_streaming(m_devh, &m_ctrl, uvcFrameCallback, this, 0);
    
    if (res < 0) {
        std::cerr << "Start streaming failed: " << uvc_strerror(res) << std::endl;
        m_running = false;
        return false;
    }
    
    return true;
}

bool StereoCamera::initializeUVC(int device_index) {
    uvc_error_t res;
    
    // Initialize UVC context
    res = uvc_init(&m_ctx, NULL);
    if (res < 0) {
        std::cerr << "UVC initialization failed: " << uvc_strerror(res) << std::endl;
        return false;
    }
    
    // Get device list
    uvc_device_t **device_list;
    res = uvc_get_device_list(m_ctx, &device_list);
    if (res < 0) {
        std::cerr << "Cannot get device list: " << uvc_strerror(res) << std::endl;
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    // Check if device index is valid
    int device_count = 0;
    while (device_list[device_count] != nullptr) {
        device_count++;
    }
    
    if (device_index >= device_count) {
        std::cerr << "Invalid device index: " << device_index << ", only " << device_count << " devices available" << std::endl;
        uvc_free_device_list(device_list, 1);
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    // Get device at specified index
    m_dev = device_list[device_index];
    uvc_ref_device(m_dev);
    uvc_free_device_list(device_list, 1);
    
    // Open device
    res = uvc_open(m_dev, &m_devh);
    if (res < 0) {
        std::cerr << "Cannot open device: " << uvc_strerror(res) << std::endl;
        uvc_unref_device(m_dev);
        m_dev = nullptr;
        uvc_exit(m_ctx);
        m_ctx = nullptr;
        return false;
    }
    
    return true;
}

void StereoCamera::cleanupUVC() {
    // Stop stream
    if (m_running) {
        uvc_stop_streaming(m_devh);
        m_running = false;
    }
    
    // Close device
    if (m_devh) {
        uvc_close(m_devh);
        m_devh = nullptr;
    }
    
    // Clean up device reference
    if (m_dev) {
        uvc_unref_device(m_dev);
        m_dev = nullptr;
    }
    
    // Exit UVC context
    if (m_ctx) {
        uvc_exit(m_ctx);
        m_ctx = nullptr;
    }
}