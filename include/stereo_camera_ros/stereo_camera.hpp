#ifndef STEREO_CAMERA_HPP
#define STEREO_CAMERA_HPP

#include <libuvc/libuvc.h>
#include <opencv2/opencv.hpp>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <chrono>
#include <memory>
#include <queue>
#include <future>
#include <deque>
#include <vector>

/**
 * @brief 双目相机类，支持UVC协议的USB相机
 * 
 * 此类封装UVC相机操作，特别支持双目相机，可以将单帧图像分为左右两个图像
 * 提供回调机制，当新图像到达时，自动调用用户提供的回调函数
 */
class StereoCamera {
public:
    /**
     * @brief 定义帧回调函数类型
     * @param left 左眼图像
     * @param right 右眼图像
     * @param timestamp 时间戳 (纳秒)
     * @param user_data 用户数据指针
     */
    using FrameCallback = std::function<void(const cv::Mat& left, const cv::Mat& right, 
                                            uint64_t timestamp, void* user_data)>;
    
    /**
     * @brief 构造函数
     */
    StereoCamera();
    
    /**
     * @brief 析构函数，自动释放资源
     */
    ~StereoCamera();
    
    /**
     * @brief 打开双目相机
     * @param device_index 设备索引，默认为0
     * @return 是否成功打开
     */
    bool open(int device_index = 0);
    
    /**
     * @brief 使用指定参数打开双目相机
     * @param width 请求的宽度
     * @param height 请求的高度
     * @param fps 请求的帧率
     * @param use_mjpeg 是否使用MJPEG格式
     * @param device_index 设备索引
     * @param worker_threads 工作线程数，默认为4
     * @return 是否成功打开
     */
    bool openWithParams(int width, int height, int fps, bool use_mjpeg = true, 
                       int device_index = 0, int worker_threads = 4);
    
    /**
     * @brief 关闭相机
     */
    void close();
    
    /**
     * @brief 检查相机是否已打开
     * @return 相机是否已打开
     */
    bool isOpened() const;
    
    /**
     * @brief 设置帧回调函数
     * @param callback 回调函数
     * @param user_data 用户数据指针
     */
    void setFrameCallback(FrameCallback callback, void* user_data = nullptr);
    
    /**
     * @brief 打印设备信息
     */
    void printDeviceInfo() const;
    
    /**
     * @brief 获取当前相机宽度
     * @return 宽度 (像素)
     */
    int getWidth() const;
    
    /**
     * @brief 获取当前相机高度
     * @return 高度 (像素)
     */
    int getHeight() const;
    
    /**
     * @brief 获取当前帧率
     * @return 帧率 (fps)
     */
    int getFps() const;
    
    /**
     * @brief 检查是否使用MJPEG模式
     * @return 是否MJPEG模式
     */
    bool isMjpegMode() const;
    
    /**
     * @brief 获取最新的左右眼帧 (非回调方式)
     * @param left 左眼图像输出
     * @param right 右眼图像输出
     * @param timestamp 时间戳输出 (纳秒)
     * @param timeout_ms 超时时间 (毫秒)
     * @return 是否成功获取新帧
     */
    bool getLatestFrames(cv::Mat& left, cv::Mat& right, uint64_t& timestamp, 
                         int timeout_ms = 1000);
    
    /**
     * @brief 设置双目分割比例
     * @param split_ratio 分割比例 (0.5表示中点分割)
     */
    void setStereoSplitRatio(float split_ratio);

    /**
     * @brief 设置丢帧阈值，当队列中帧数超过此阈值时，丢弃旧帧
     * @param threshold 帧数阈值
     */
    void setFrameDropThreshold(size_t threshold);
    
    /**
     * @brief 获取当前处理队列长度
     * @return 队列长度
     */
    size_t getQueueLength() const;
    
private:
    // 相机参数
    int m_width;
    int m_height;
    int m_fps;
    bool m_use_mjpeg;
    float m_split_ratio;  // 双目分割比例
    
    // UVC相关变量
    uvc_context_t* m_ctx;
    uvc_device_t* m_dev;
    uvc_device_handle_t* m_devh;
    uvc_stream_ctrl_t m_ctrl;
    
    // 回调相关
    FrameCallback m_callback;
    void* m_user_data;
    
    // 线程控制
    std::atomic<bool> m_running;
    std::atomic<bool> m_is_opened;
    mutable std::mutex m_mutex;  // 通用锁
    
    // 帧缓存
    mutable std::mutex m_frame_mutex;
    std::condition_variable m_frame_cond;
    cv::Mat m_latest_left;
    cv::Mat m_latest_right;
    uint64_t m_latest_timestamp;
    bool m_new_frame_available;
    std::deque<uint64_t> m_frame_timestamp_history;  // 帧率历史记录

    // 多线程处理队列
    struct FrameData {
        uvc_frame_t* frame;
        uint64_t timestamp;
    };
    mutable std::mutex m_queue_mutex;
    std::condition_variable m_queue_cond;
    std::queue<FrameData> m_frame_queue;
    std::vector<std::thread> m_worker_threads;
    std::atomic<bool> m_workers_running;
    size_t m_drop_threshold;  // 丢帧阈值
    std::atomic<size_t> m_dropped_frames;  // 丢弃的帧数

    // 预分配转换缓冲区
    std::vector<cv::Mat> m_conversion_buffers;
    mutable std::mutex m_buffer_mutex;
    
    // UVC回调函数
    static void uvcFrameCallback(uvc_frame_t* frame, void* ptr);
    
    // 处理帧
    void processFrame(uvc_frame_t* frame);

    // 工作线程函数
    void workerThread();
    
    // 将整帧拆分为左右眼帧
    void splitStereoFrame(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right) const;
    
    // 从MJPEG解码到BGR格式 (优化版本)
    cv::Mat mjpegToBgr(const uint8_t* mjpeg_data, size_t mjpeg_size, cv::Mat& buffer) const;
    
    // 从YUYV转换到BGR格式
    cv::Mat yuyvToBgr(const uint8_t* yuyv_data, int width, int height) const;
    
    // 检查相机支持的格式
    bool checkCameraFormat();
    
    // 初始化和清理
    bool initializeUVC(int device_index);
    void cleanupUVC();
    void startWorkerThreads(int thread_count);
    void stopWorkerThreads();
    
    // 禁用复制构造函数和赋值运算符
    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;
};

#endif // STEREO_CAMERA_HPP
