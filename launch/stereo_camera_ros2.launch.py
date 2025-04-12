from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    device_index = LaunchConfiguration('device_index', default='0')
    width = LaunchConfiguration('width', default='3840')
    height = LaunchConfiguration('height', default='1080')
    fps = LaunchConfiguration('fps', default='60')
    use_mjpeg = LaunchConfiguration('use_mjpeg', default='true')
    split_ratio = LaunchConfiguration('split_ratio', default='0.5')
    
    left_camera_name = LaunchConfiguration('left_camera_name', default='left_camera')
    right_camera_name = LaunchConfiguration('right_camera_name', default='right_camera')
    left_frame_id = LaunchConfiguration('left_frame_id', default='left_camera_optical_frame')
    right_frame_id = LaunchConfiguration('right_frame_id', default='right_camera_optical_frame')
    
    left_image_topic = LaunchConfiguration('left_image_topic', default='/left_camera')
    right_image_topic = LaunchConfiguration('right_image_topic', default='/right_camera')
    
    left_camera_info_url = LaunchConfiguration('left_camera_info_url', default='')
    right_camera_info_url = LaunchConfiguration('right_camera_info_url', default='')
    
    # 参数声明
    declare_device_index = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='Camera device index')
    
    declare_width = DeclareLaunchArgument(
        'width',
        default_value='3840',
        description='Image width')
    
    declare_height = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Image height')
    
    declare_fps = DeclareLaunchArgument(
        'fps',
        default_value='60',
        description='Camera framerate')
    
    declare_use_mjpeg = DeclareLaunchArgument(
        'use_mjpeg',
        default_value='true',
        description='Use MJPEG format')
    
    declare_split_ratio = DeclareLaunchArgument(
        'split_ratio',
        default_value='0.5',
        description='Stereo split ratio')
    
    declare_left_camera_name = DeclareLaunchArgument(
        'left_camera_name',
        default_value='left_camera',
        description='Left camera name')
    
    declare_right_camera_name = DeclareLaunchArgument(
        'right_camera_name',
        default_value='right_camera',
        description='Right camera name')
    
    declare_left_frame_id = DeclareLaunchArgument(
        'left_frame_id',
        default_value='left_camera_optical_frame',
        description='Left camera frame ID')
    
    declare_right_frame_id = DeclareLaunchArgument(
        'right_frame_id',
        default_value='right_camera_optical_frame',
        description='Right camera frame ID')
    
    declare_left_image_topic = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/left_camera',
        description='Left camera image topic')
    
    declare_right_image_topic = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/right_camera',
        description='Right camera image topic')
    
    declare_left_camera_info_url = DeclareLaunchArgument(
        'left_camera_info_url',
        default_value='',
        description='URL to left camera calibration file')
    
    declare_right_camera_info_url = DeclareLaunchArgument(
        'right_camera_info_url',
        default_value='',
        description='URL to right camera calibration file')
    
    # 启动相机节点
    stereo_camera_node = Node(
        package='stereo_camera_ros',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'device_index': device_index,
            'width': width,
            'height': height,
            'fps': fps,
            'use_mjpeg': use_mjpeg,
            'split_ratio': split_ratio,
            'left_camera_name': left_camera_name,
            'right_camera_name': right_camera_name,
            'left_frame_id': left_frame_id,
            'right_frame_id': right_frame_id,
            'left_image_topic': left_image_topic,
            'right_image_topic': right_image_topic,
            'left_camera_info_url': left_camera_info_url,
            'right_camera_info_url': right_camera_info_url
        }]
    )
    
    return LaunchDescription([
        declare_device_index,
        declare_width,
        declare_height,
        declare_fps,
        declare_use_mjpeg,
        declare_split_ratio,
        declare_left_camera_name,
        declare_right_camera_name,
        declare_left_frame_id,
        declare_right_frame_id,
        declare_left_image_topic,
        declare_right_image_topic,
        declare_left_camera_info_url,
        declare_right_camera_info_url,
        stereo_camera_node
    ])