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
 * @brief Stereo camera class, supports USB cameras with UVC protocol
 * 
 * This class encapsulates UVC camera operations, specifically supports stereo cameras,
 * can split a single frame image into left and right images.
 * Provides a callback mechanism that automatically calls user-provided callback function 
 * when new images arrive
 */
class StereoCamera {
public:
    /**
     * @brief Defines the frame callback function type
     * @param left Left eye image
     * @param right Right eye image
     * @param timestamp Timestamp (nanoseconds)
     * @param user_data User data pointer
     */
    using FrameCallback = std::function<void(const cv::Mat& left, const cv::Mat& right, 
                                            uint64_t timestamp, void* user_data)>;
    
    /**
     * @brief Constructor
     */
    StereoCamera();
    
    /**
     * @brief Destructor, automatically releases resources
     */
    ~StereoCamera();
    
    /**
     * @brief Open stereo camera
     * @param device_index Device index, defaults to 0
     * @return Whether successfully opened
     */
    bool open(int device_index = 0);
    
    /**
     * @brief Open stereo camera with specified parameters
     * @param width Requested width
     * @param height Requested height
     * @param fps Requested frame rate
     * @param use_mjpeg Whether to use MJPEG format
     * @param device_index Device index
     * @param worker_threads Number of worker threads, defaults to 4
     * @return Whether successfully opened
     */
    bool openWithParams(int width, int height, int fps, bool use_mjpeg = true, 
                       int device_index = 0, int worker_threads = 4);
    
    /**
     * @brief Close the camera
     */
    void close();
    
    /**
     * @brief Check if camera is opened
     * @return Whether camera is opened
     */
    bool isOpened() const;
    
    /**
     * @brief Set frame callback function
     * @param callback Callback function
     * @param user_data User data pointer
     */
    void setFrameCallback(FrameCallback callback, void* user_data = nullptr);
    
    /**
     * @brief Print device information
     */
    void printDeviceInfo() const;
    
    /**
     * @brief Get current camera width
     * @return Width (pixels)
     */
    int getWidth() const;
    
    /**
     * @brief Get current camera height
     * @return Height (pixels)
     */
    int getHeight() const;
    
    /**
     * @brief Get current frame rate
     * @return Frame rate (fps)
     */
    int getFps() const;
    
    /**
     * @brief Check if using MJPEG mode
     * @return Whether MJPEG mode is used
     */
    bool isMjpegMode() const;
    
    /**
     * @brief Get latest left and right frames (non-callback method)
     * @param left Left eye image output
     * @param right Right eye image output
     * @param timestamp Timestamp output (nanoseconds)
     * @param timeout_ms Timeout (milliseconds)
     * @return Whether successfully got new frame
     */
    bool getLatestFrames(cv::Mat& left, cv::Mat& right, uint64_t& timestamp, 
                         int timeout_ms = 1000);
    
    /**
     * @brief Set stereo split ratio
     * @param split_ratio Split ratio (0.5 means split at middle point)
     */
    void setStereoSplitRatio(float split_ratio);

    /**
     * @brief Set frame drop threshold, discard old frames when queue frames exceed this threshold
     * @param threshold Frame threshold
     */
    void setFrameDropThreshold(size_t threshold);
    
    /**
     * @brief Get current processing queue length
     * @return Queue length
     */
    size_t getQueueLength() const;
    
private:
    // Camera parameters
    int m_width;
    int m_height;
    int m_fps;
    bool m_use_mjpeg;
    float m_split_ratio;  // Stereo split ratio
    
    // UVC related variables
    uvc_context_t* m_ctx;
    uvc_device_t* m_dev;
    uvc_device_handle_t* m_devh;
    uvc_stream_ctrl_t m_ctrl;
    
    // Callback related
    FrameCallback m_callback;
    void* m_user_data;
    
    // Thread control
    std::atomic<bool> m_running;
    std::atomic<bool> m_is_opened;
    mutable std::mutex m_mutex;  // General lock
    
    // Frame cache
    mutable std::mutex m_frame_mutex;
    std::condition_variable m_frame_cond;
    cv::Mat m_latest_left;
    cv::Mat m_latest_right;
    uint64_t m_latest_timestamp;
    bool m_new_frame_available;
    std::deque<uint64_t> m_frame_timestamp_history;  // Frame rate history record

    // Multi-threaded processing queue
    struct FrameData {
        uvc_frame_t* frame;
        uint64_t timestamp;
    };
    mutable std::mutex m_queue_mutex;
    std::condition_variable m_queue_cond;
    std::queue<FrameData> m_frame_queue;
    std::vector<std::thread> m_worker_threads;
    std::atomic<bool> m_workers_running;
    size_t m_drop_threshold;  // Frame drop threshold
    std::atomic<size_t> m_dropped_frames;  // Number of dropped frames

    // Pre-allocated conversion buffers
    std::vector<cv::Mat> m_conversion_buffers;
    mutable std::mutex m_buffer_mutex;
    
    // UVC callback function
    static void uvcFrameCallback(uvc_frame_t* frame, void* ptr);
    
    // Process frame
    void processFrame(uvc_frame_t* frame);

    // Worker thread function
    void workerThread();
    
    // Split stereo frame into left and right eye frames
    void splitStereoFrame(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right) const;
    
    // Decode from MJPEG to BGR format (optimized version)
    cv::Mat mjpegToBgr(const uint8_t* mjpeg_data, size_t mjpeg_size, cv::Mat& buffer) const;
    
    // Convert from YUYV to BGR format
    cv::Mat yuyvToBgr(const uint8_t* yuyv_data, int width, int height) const;
    
    // Check camera supported formats
    bool checkCameraFormat();
    
    // Initialization and cleanup
    bool initializeUVC(int device_index);
    void cleanupUVC();
    void startWorkerThreads(int thread_count);
    void stopWorkerThreads();
    
    // Disable copy constructor and assignment operator
    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;
};

#endif // STEREO_CAMERA_HPP
