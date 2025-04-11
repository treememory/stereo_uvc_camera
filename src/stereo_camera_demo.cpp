#include "stereo_camera.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <signal.h>

// 全局变量用于处理信号
std::atomic<bool> g_running(true);

// 定义帧结构体，存储左右眼图像和时间戳
struct StereoFrame {
    cv::Mat left;
    cv::Mat right;
    uint64_t timestamp;
    std::chrono::steady_clock::time_point capture_time;
};

// 帧队列和同步对象
std::queue<StereoFrame> g_frame_queue;
std::mutex g_queue_mutex;
std::condition_variable g_queue_condition;
const size_t MAX_QUEUE_SIZE = 5; // 最大队列大小，防止内存溢出

// FPS统计
std::atomic<int> g_frame_count(0);
std::atomic<float> g_current_fps(0.0f);
std::chrono::steady_clock::time_point g_fps_last_time;

// 信号处理函数
void signalHandler(int sig) {
    std::cout << "接收到信号 " << sig << ", 正在退出..." << std::endl;
    g_running = false;
    g_queue_condition.notify_all(); // 通知等待的线程退出
}

// 帧回调函数 - 现在只负责将帧放入队列，不再显示图像
void frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data) {
    // 增加帧计数
    g_frame_count++;
    
    // 每秒更新一次FPS
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_fps_last_time).count();
    if (elapsed >= 1000) {
        g_current_fps = g_frame_count * 1000.0f / elapsed;
        g_frame_count = 0;
        g_fps_last_time = now;
    }
    
    // 创建帧结构体
    StereoFrame frame;
    left.copyTo(frame.left);
    right.copyTo(frame.right);
    frame.timestamp = timestamp;
    frame.capture_time = now;
    
    // 加锁访问共享队列
    {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        
        // 如果队列已满，移除最旧的帧
        while (g_frame_queue.size() >= MAX_QUEUE_SIZE) {
            g_frame_queue.pop();
            std::cout << "警告: 显示线程延迟，丢弃旧帧" << std::endl;
        }
        
        // 添加新帧到队列
        g_frame_queue.push(frame);
    }
    
    // 通知显示线程有新帧可用
    g_queue_condition.notify_one();
}

// 显示线程函数 - 专门负责图像显示
void displayThreadFunction() {
    // 创建显示窗口
    cv::namedWindow("左眼图像", cv::WINDOW_NORMAL);
    cv::namedWindow("右眼图像", cv::WINDOW_NORMAL);
    
    while (g_running) {
        StereoFrame frame;
        bool has_frame = false;
        
        // 从队列中获取帧
        {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            
            // 等待新帧或退出信号
            g_queue_condition.wait_for(lock, std::chrono::milliseconds(100), []{
                return !g_frame_queue.empty() || !g_running;
            });
            
            // 再次检查是否需要退出
            if (!g_running && g_frame_queue.empty()) {
                break;
            }
            
            // 获取帧
            if (!g_frame_queue.empty()) {
                frame = g_frame_queue.front();
                g_frame_queue.pop();
                has_frame = true;
            }
        }
        
        // 处理和显示图像
        if (has_frame) {
            // 计算延迟
            auto now = std::chrono::steady_clock::now();
            double display_delay = std::chrono::duration<double, std::milli>(now - frame.capture_time).count();
            
            // 格式化时间戳
            uint64_t ts_sec = frame.timestamp / 1000000000ULL;
            uint64_t ts_nsec = frame.timestamp % 1000000000ULL;
            char ts_str[64];
            sprintf(ts_str, "时间戳: %lu.%09lu", ts_sec, ts_nsec);
            
            // 添加信息到图像
            cv::putText(frame.left, ts_str, cv::Point(20, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.left, "FPS: " + std::to_string(g_current_fps), cv::Point(20, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.left, "显示延迟: " + std::to_string(display_delay) + "ms", cv::Point(20, 90), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.right, "右眼图像", cv::Point(20, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            
            // 显示图像
            cv::imshow("左眼图像", frame.left);
            cv::imshow("右眼图像", frame.right);
            
            // 处理键盘事件，使用较短的等待时间，以保持UI响应性
            int key = cv::waitKey(1);
            if (key == 27) { // ESC键
                g_running = false;
            }
        }
    }
    
    cv::destroyWindow("左眼图像");
    cv::destroyWindow("右眼图像");
}

int main(int argc, char** argv) {
    // 初始化FPS计时器
    g_fps_last_time = std::chrono::steady_clock::now();
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 创建相机对象
    StereoCamera camera;
    
    // 解析命令行参数
    int device_index = 0;
    int width = 3840;
    int height = 1080;
    int fps = 60;
    bool use_mjpeg = true;
    float split_ratio = 0.5f;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-d" && i + 1 < argc) {
            device_index = std::stoi(argv[++i]);
        }
        else if (arg == "-w" && i + 1 < argc) {
            width = std::stoi(argv[++i]);
        }
        else if (arg == "-h" && i + 1 < argc) {
            height = std::stoi(argv[++i]);
        }
        else if (arg == "-f" && i + 1 < argc) {
            fps = std::stoi(argv[++i]);
        }
        else if (arg == "-m" && i + 1 < argc) {
            use_mjpeg = std::stoi(argv[++i]) != 0;
        }
        else if (arg == "-s" && i + 1 < argc) {
            split_ratio = std::stof(argv[++i]);
        }
        else if (arg == "--help") {
            std::cout << "双目相机演示程序" << std::endl;
            std::cout << "用法: " << argv[0] << " [选项]" << std::endl;
            std::cout << "选项:" << std::endl;
            std::cout << "  -d <index>     设备索引 (默认: 0)" << std::endl;
            std::cout << "  -w <width>     宽度 (默认: 1920)" << std::endl;
            std::cout << "  -h <height>    高度 (默认: 1080)" << std::endl;
            std::cout << "  -f <fps>       帧率 (默认: 30)" << std::endl;
            std::cout << "  -m <0|1>       使用MJPEG格式 (默认: 1)" << std::endl;
            std::cout << "  -s <ratio>     双目分割比例 (默认: 0.5)" << std::endl;
            std::cout << "  --help         显示帮助" << std::endl;
            return 0;
        }
    }
    
    // 设置双目分割比例
    camera.setStereoSplitRatio(split_ratio);
    
    // 打开相机
    std::cout << "正在打开相机..." << std::endl;
    if (!camera.openWithParams(width, height, fps, use_mjpeg, device_index)) {
        std::cerr << "无法打开相机" << std::endl;
        return -1;
    }
    
    // 打印设备信息
    camera.printDeviceInfo();
    
    // 设置帧回调
    camera.setFrameCallback(frameCallback);
    
    // 启动显示线程
    std::thread display_thread(displayThreadFunction);
    
    // 主线程等待退出信号
    std::cout << "相机已打开，按Ctrl+C或ESC退出" << std::endl;
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 清理资源
    camera.close();
    
    // 等待显示线程结束
    if (display_thread.joinable()) {
        display_thread.join();
    }
    
    std::cout << "程序已退出" << std::endl;
    return 0;
}
