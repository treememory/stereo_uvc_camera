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

// Global variables for signal handling
std::atomic<bool> g_running(true);

// Frame structure definition, stores left and right eye images and timestamps
struct StereoFrame {
    cv::Mat left;
    cv::Mat right;
    uint64_t timestamp;
    std::chrono::steady_clock::time_point capture_time;
};

// Frame queue and synchronization objects
std::queue<StereoFrame> g_frame_queue;
std::mutex g_queue_mutex;
std::condition_variable g_queue_condition;
const size_t MAX_QUEUE_SIZE = 5; // Maximum queue size to prevent memory overflow

// FPS statistics
std::atomic<int> g_frame_count(0);
std::atomic<float> g_current_fps(0.0f);
std::chrono::steady_clock::time_point g_fps_last_time;

// Signal handler function
void signalHandler(int sig) {
    std::cout << "Signal received " << sig << ", exiting..." << std::endl;
    g_running = false;
    g_queue_condition.notify_all(); // Notify waiting threads to exit
}

// Frame callback function - now only responsible for putting frames in the queue, no longer displaying images
void frameCallback(const cv::Mat& left, const cv::Mat& right, uint64_t timestamp, void* user_data) {
    // Increment frame count
    g_frame_count++;
    
    // Update FPS once per second
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_fps_last_time).count();
    if (elapsed >= 1000) {
        g_current_fps = g_frame_count * 1000.0f / elapsed;
        g_frame_count = 0;
        g_fps_last_time = now;
    }
    
    // Create frame structure
    StereoFrame frame;
    left.copyTo(frame.left);
    right.copyTo(frame.right);
    frame.timestamp = timestamp;
    frame.capture_time = now;
    
    // Lock to access shared queue
    {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        
        // If the queue is full, remove the oldest frame
        while (g_frame_queue.size() >= MAX_QUEUE_SIZE) {
            g_frame_queue.pop();
            std::cout << "Warning: Display thread delay, dropping old frame" << std::endl;
        }
        
        // Add new frame to the queue
        g_frame_queue.push(frame);
    }
    
    // Notify the display thread that a new frame is available
    g_queue_condition.notify_one();
}

// Display thread function - specifically responsible for image display
void displayThreadFunction() {
    // Create display windows
    cv::namedWindow("Left Eye Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Right Eye Image", cv::WINDOW_NORMAL);
    
    while (g_running) {
        StereoFrame frame;
        bool has_frame = false;
        
        // Get frame from queue
        {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            
            // Wait for new frame or exit signal
            g_queue_condition.wait_for(lock, std::chrono::milliseconds(100), []{
                return !g_frame_queue.empty() || !g_running;
            });
            
            // Check again if we need to exit
            if (!g_running && g_frame_queue.empty()) {
                break;
            }
            
            // Get frame
            if (!g_frame_queue.empty()) {
                frame = g_frame_queue.front();
                g_frame_queue.pop();
                has_frame = true;
            }
        }
        
        // Process and display images
        if (has_frame) {
            // Calculate delay
            auto now = std::chrono::steady_clock::now();
            double display_delay = std::chrono::duration<double, std::milli>(now - frame.capture_time).count();
            
            // Format timestamp
            uint64_t ts_sec = frame.timestamp / 1000000000ULL;
            uint64_t ts_nsec = frame.timestamp % 1000000000ULL;
            char ts_str[64];
            sprintf(ts_str, "Timestamp: %lu.%09lu", ts_sec, ts_nsec);
            
            // Add information to the image
            cv::putText(frame.left, ts_str, cv::Point(20, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.left, "FPS: " + std::to_string(g_current_fps), cv::Point(20, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.left, "Display delay: " + std::to_string(display_delay) + "ms", cv::Point(20, 90), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame.right, "Right Eye Image", cv::Point(20, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            
            // Display images
            cv::imshow("Left Eye Image", frame.left);
            cv::imshow("Right Eye Image", frame.right);
            
            // Handle keyboard events, use shorter wait time to keep UI responsive
            int key = cv::waitKey(1);
            if (key == 27) { // ESC key
                g_running = false;
            }
        }
    }
    
    cv::destroyWindow("Left Eye Image");
    cv::destroyWindow("Right Eye Image");
}

int main(int argc, char** argv) {
    // Initialize FPS timer
    g_fps_last_time = std::chrono::steady_clock::now();
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Create camera object
    StereoCamera camera;
    
    // Parse command line arguments
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
            std::cout << "Stereo Camera Demo Program" << std::endl;
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  -d <index>     Device index (default: 0)" << std::endl;
            std::cout << "  -w <width>     Width (default: 1920)" << std::endl;
            std::cout << "  -h <height>    Height (default: 1080)" << std::endl;
            std::cout << "  -f <fps>       Frame rate (default: 30)" << std::endl;
            std::cout << "  -m <0|1>       Use MJPEG format (default: 1)" << std::endl;
            std::cout << "  -s <ratio>     Stereo split ratio (default: 0.5)" << std::endl;
            std::cout << "  --help         Show help" << std::endl;
            return 0;
        }
    }
    
    // Set stereo split ratio
    camera.setStereoSplitRatio(split_ratio);
    
    // Open camera
    std::cout << "Opening camera..." << std::endl;
    if (!camera.openWithParams(width, height, fps, use_mjpeg, device_index)) {
        std::cerr << "Unable to open camera" << std::endl;
        return -1;
    }
    
    // Print device information
    camera.printDeviceInfo();
    
    // Set frame callback
    camera.setFrameCallback(frameCallback);
    
    // Start display thread
    std::thread display_thread(displayThreadFunction);
    
    // Main thread waits for exit signal
    std::cout << "Camera is open, press Ctrl+C or ESC to exit" << std::endl;
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean up resources
    camera.close();
    
    // Wait for display thread to end
    if (display_thread.joinable()) {
        display_thread.join();
    }
    
    std::cout << "Program has exited" << std::endl;
    return 0;
}
