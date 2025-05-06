# bash 脚本录制 rosbag

# 1. 录制 rosbag
rosbag record -o ./bags/throttled.bag \
    /stereo_camera/left/throttled \
    /stereo_camera/right/throttled