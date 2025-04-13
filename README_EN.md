# Stereo Camera ROS

[简体中文](README.md) | English

A high-performance USB stereo camera ROS driver supporting both ROS1 and ROS2 platforms, specially optimized for image processing at high frame rates.

## Features
- Support for common USB stereo camera devices (UVC protocol)
- Compatible with both ROS1 and ROS2 platforms
- Efficient multi-threaded image processing mechanism
- Automatic left/right image splitting functionality
- Support for MJPEG and YUYV formats, prioritizing MJPEG for higher frame rates
- Performance optimization at high frame rates to minimize frame dropping
- Dynamic parameter configuration
- Flexible camera parameter settings

## Performance Optimization
This driver is specially optimized for image processing in high frame rate scenarios (50fps+):
- **Multi-threaded parallel processing**: Uses thread pools for image decoding and conversion, avoiding single-thread bottlenecks
- **Memory pre-allocation**: Reduces runtime memory allocation overhead
- **Queue management**: Intelligent frame queue management, ensuring orderly frame dropping when processing speed falls behind
- **MJPEG hardware acceleration**: Fully utilizes OpenCV's hardware-accelerated decoding functionality

## Dependencies
- ROS1 (Noetic and above) or ROS2 (Foxy and above)
- OpenCV (≥4.2.0)
- libuvc (≥0.0.6)
- libjpeg

## Installation
### Installing Dependencies
```bash
# Ubuntu system
sudo apt-get update
sudo apt-get install -y libopencv-dev libjpeg-dev libuvc-dev
```

### Building the Driver
#### ROS1
```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your-username/stereo_camera_ros.git
cd ..
# Build
catkin_make
source devel/setup.bash
```

#### ROS2
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/stereo_camera_ros.git
cd ..
# Build
colcon build --symlink-install
source install/setup.bash
```

## Quick Start
### Launching the Camera Node
```bash
# ROS1
roslaunch stereo_camera_ros stereo_camera.launch

# ROS2
ros2 launch stereo_camera_ros stereo_camera.launch.py
```

### Parameter Configuration
You can set camera parameters by modifying the launch file or via command line parameters:
```bash
# ROS1
roslaunch stereo_camera_ros stereo_camera.launch width:=1280 height:=480 fps:=60 device_index:=0

# ROS2
ros2 launch stereo_camera_ros stereo_camera.launch.py width:=1280 height:=480 fps:=60 device_index:=0
```

## Key Parameters
| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `device_index` | int | 0 | USB camera device index |
| `width` | int | 1920 | Requested image width |
| `height` | int | 1080 | Requested image height |
| `fps` | int | 30 | Requested frame rate |
| `use_mjpeg` | bool | true | Whether to use MJPEG format |
| `split_ratio` | float | 0.5 | Left/right image split ratio (0.1-0.9) |
| `worker_threads` | int | 4 | Number of image processing worker threads |
| `frame_drop_threshold` | int | 10 | Frame dropping threshold, old frames are dropped when queue exceeds this length |

## Published Topics
- `/stereo_camera/left/image_raw` - Left eye raw image (sensor_msgs/Image)
- `/stereo_camera/right/image_raw` - Right eye raw image (sensor_msgs/Image)
- `/stereo_camera/left/camera_info` - Left eye camera info (sensor_msgs/CameraInfo)
- `/stereo_camera/right/camera_info` - Right eye camera info (sensor_msgs/CameraInfo)

## Performance Tuning
### Handling High Frame Rates
When setting a high frame rate (e.g., 50fps) but actual processing frame rate falls short, you can:
1. Increase the number of worker threads:
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch worker_threads:=8
   ```
2. Adjust the frame dropping threshold to control memory usage:
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch frame_drop_threshold:=5
   ```
3. Ensure MJPEG mode is used to reduce bandwidth requirements:
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch use_mjpeg:=true
   ```

### Hardware Acceleration
Ensure your system supports hardware-accelerated JPEG decoding:
- On systems with CUDA support, ensure OpenCV was compiled with CUDA enabled
- On Intel processors, the IPP library can accelerate image processing

## Troubleshooting
### Common Issues
1. **Cannot open camera device**
   - Check device permissions: `sudo chmod a+rw /dev/video*`
   - Confirm the device index is correct
   - Use `v4l2-ctl --list-devices` to see available devices
   
2. **Frame rate lower than expected**
   - Check USB bandwidth, preferably use a USB 3.0+ interface
   - Increase the number of worker threads
   - Reduce resolution or use MJPEG format
   
3. **Incorrect image splitting**
   - Adjust the `split_ratio` parameter to change the left/right split ratio

## License
MIT

## Contributions
Pull Requests and Issues are welcome!

## Author
Rock

## Acknowledgements
- libuvc development team
- OpenCV community
- ROS community