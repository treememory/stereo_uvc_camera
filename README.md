# Stereo Camera ROS

简体中文 | [English](README_EN.md)

一个高性能的USB双目相机ROS驱动，支持ROS1和ROS2平台，特别针对高帧率下的图像处理进行了优化。

## 功能特点

- 支持通用USB双目相机设备 (UVC协议)
- 支持ROS1和ROS2双平台
- 高效的多线程图像处理机制
- 自动左右图像分割功能
- 支持MJPEG和YUYV格式，优先使用MJPEG以获得更高的帧率
- 高帧率下的性能优化，最大程度减少帧丢失
- 动态参数配置
- 灵活的相机参数设置

## 性能优化

本驱动针对高帧率场景（如50fps+）下的图像处理进行了专门优化：

- **多线程并行处理**：使用线程池处理图像解码和转换，避免单线程处理瓶颈
- **内存预分配**：减少运行时内存分配开销
- **队列管理**：智能帧队列管理，确保当处理速度跟不上时有序丢帧
- **MJPEG硬件加速**：充分利用OpenCV的硬件加速解码功能

## 依赖项

- ROS1 (Noetic及以上) 或 ROS2 (Foxy及以上)
- OpenCV (≥4.2.0)
- libuvc (≥0.0.6)
- libjpeg

## 安装

### 安装依赖项

```bash
# Ubuntu系统
sudo apt-get update
sudo apt-get install -y libopencv-dev libjpeg-dev libuvc-dev

### 编译驱动

#### ROS1

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your-username/stereo_camera_ros.git
cd ..

# 编译
catkin_make
source devel/setup.bash
```

#### ROS2

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/stereo_camera_ros.git
cd ..

# 编译
colcon build --symlink-install
source install/setup.bash
```

## 快速开始

### 启动相机节点

```bash
# ROS1
roslaunch stereo_camera_ros stereo_camera.launch

# ROS2
ros2 launch stereo_camera_ros stereo_camera.launch.py
```

### 参数配置

可以通过修改launch文件或通过命令行参数设置相机参数：

```bash
# ROS1
roslaunch stereo_camera_ros stereo_camera.launch width:=1280 height:=480 fps:=60 device_index:=0

# ROS2
ros2 launch stereo_camera_ros stereo_camera.launch.py width:=1280 height:=480 fps:=60 device_index:=0
```

## 关键参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|-------|------|
| `device_index` | int | 0 | USB相机设备索引 |
| `width` | int | 1920 | 请求的图像宽度 |
| `height` | int | 1080 | 请求的图像高度 |
| `fps` | int | 30 | 请求的帧率 |
| `use_mjpeg` | bool | true | 是否使用MJPEG格式 |
| `split_ratio` | float | 0.5 | 左右图像分割比例 (0.1-0.9) |
| `worker_threads` | int | 4 | 图像处理工作线程数 |
| `frame_drop_threshold` | int | 10 | 帧丢弃阈值，队列超过此长度时丢弃旧帧 |

## 发布的话题

- `/stereo_camera/left/image_raw` - 左眼原始图像 (sensor_msgs/Image)
- `/stereo_camera/right/image_raw` - 右眼原始图像 (sensor_msgs/Image)
- `/stereo_camera/left/camera_info` - 左眼相机信息 (sensor_msgs/CameraInfo)
- `/stereo_camera/right/camera_info` - 右眼相机信息 (sensor_msgs/CameraInfo)

## 性能调优

### 处理高帧率

当设置高帧率(如50fps)而实际处理帧率达不到时，可以：

1. 增加工作线程数量：
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch worker_threads:=8
   ```

2. 调整帧丢弃阈值，控制内存使用：
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch frame_drop_threshold:=5
   ```

3. 确保使用MJPEG模式以减少带宽需求：
   ```bash
   roslaunch stereo_camera_ros stereo_camera.launch use_mjpeg:=true
   ```

### 硬件加速

确保系统支持硬件加速的JPEG解码：
- 在支持CUDA的系统上，确保OpenCV编译时启用了CUDA支持
- 在Intel处理器上，IPP库可以加速图像处理

## 故障排除

### 常见问题

1. **无法打开相机设备**
   - 检查设备权限: `sudo chmod a+rw /dev/video*`
   - 确认设备索引是否正确
   - 使用`v4l2-ctl --list-devices`查看可用设备

2. **帧率低于预期**
   - 检查USB带宽，尽量使用USB 3.0+接口
   - 增加工作线程数
   - 减小分辨率或使用MJPEG格式

3. **图像分割不正确**
   - 调整`split_ratio`参数以改变左右分割比例

## 许可证

MIT

## 贡献

欢迎提交Pull Request或创建Issue！

## 作者

Rock

## 致谢

- libuvc开发团队
- OpenCV社区
- ROS社区