import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 使用示例：
# 真实摄像头 + 真实串口（需指定串口设备）
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=false serial_sim:=false port:=/dev/ttyUSB0 camera_id:=4
# 真实摄像头 + 串口模拟（无物理串口）
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=false serial_sim:=true
# 模拟图像（合成 tag）+ 串口模拟
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=true serial_sim:=true
# 只跑 tag 检测测距（不开 IK/串口）：serial_sim 对 cam_pos 无影响，IK 节点始终会启动

# 有时候 相机的id会变，请使用下面的命令查看设备
# ls /dev/video*

# 超级好用的摄像头调试工具
# sudo apt install guvcview
# guvcview -d /dev/video4

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='true')
    serial_sim = LaunchConfiguration('serial_sim', default='true')

    # --- 摄像头参数 ---
    # 这里的camera_id width ... 一律不设默认值，因为会被后面的默认值覆盖，第61行
    camera_id = LaunchConfiguration('camera_id')      # 设备号（对应 /dev/video4）
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    freq = LaunchConfiguration('freq')            # 发布帧率

    # --- 串口端口 ---
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    # 杆长不再从 launch 传：myik / myfk 节点里的默认值就是实测值，保持单一来源，
    # 避免出现 launch 写 300/300、代码里是 205.23 这种两边对不上的情况。

    # --- tag 参数 ---
    tag_size = LaunchConfiguration('tag_size', default='40.0')    # tag 真实边长 (mm)
    tag_id = LaunchConfiguration('tag_id', default='1')

    calib_file = os.path.join(
        get_package_share_directory('cv'), 'config', 'gc480p.json')

    return LaunchDescription([
          DeclareLaunchArgument('use_sim', default_value='true',
                                   description='true=合成tag图像(tag_image_pub), false=真摄像头(raw_image_pub)'),
          DeclareLaunchArgument('serial_sim', default_value='true',
                                   description='true=串口sim模式(无设备也能跑, 仅打印帧), false=真实串口'),

          # ----- 摄像头参数声明 -----
          # 30fps帧率延迟较大，应该是算力跟不上
          DeclareLaunchArgument('camera_id', default_value='4',
                                   description='摄像头设备号（对应 /dev/video 后的数字）'),
          DeclareLaunchArgument('width', default_value='640',
                                   description='图像宽度（像素）'),
          DeclareLaunchArgument('height', default_value='480',
                                   description='图像高度（像素）'),
          DeclareLaunchArgument('freq', default_value='15.0',
                                   description='发布图像的目标帧率 (Hz) '),
          DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                                   description='串口设备路径（仅 serial_sim:=false 时有效）'),

          # ----- tag 参数声明 -----
          DeclareLaunchArgument('tag_size', default_value='40.0',
                                   description='AprilTag 真实边长(mm), 相似三角形测距必需'),
          DeclareLaunchArgument('tag_id', default_value='1',
                                   description='要跟踪的 AprilTag ID <0 表示跟踪最大那个'),

          # ---- 图像来源 ----
          Node(package='cv', executable='Apriltag_image_pub', name='tag_image_pub',
               condition=IfCondition(PythonExpression(["'", use_sim, "' == 'true'"])),
               output='screen'),
          # 原自定义 raw_image_pub 节点，现替换为 ROS2 官方 usb_cam 节点（保留原代码作为注释）
          # Node(package='cv', executable='raw_image_pub', name='raw_image_pub',
          #      condition=IfCondition(PythonExpression(["'", use_sim, "' == 'false'"])),
          #      parameters=[{
          #           'camera_id': camera_id,
          #           'width': width,
          #           'height': height,
          #           'freq': freq,
          #      }],
          #      output='screen'),
          # 使用 usb_cam 节点驱动真实摄像头（支持硬件压缩和更多参数）
          Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
               condition=IfCondition(PythonExpression(["'", use_sim, "' == 'false'"])),
               parameters=[{
                    'video_device': PythonExpression(["'/dev/video' + '", camera_id, "'"]),
                    'pixel_format': 'mjpeg2rgb',
                    'image_width': width,
                    'image_height': height,
                    'framerate': freq,
                    'camera_name': '',   # 不加前缀
               }],
               remappings=[('/image_raw', '/camera/image_raw')],
               output='screen'),

          # ---- tag 检测 -> 相似三角形测距 -> 发布 /goal_position ----
          Node(package='cv', executable='Apriltag_pose', name='cam_pos',
               parameters=[{
                    'calib_file': calib_file,
                    'tag_size_mm': tag_size,
                    'tag_id': tag_id,
                    'frame_id': 'camera_frame',
               }],
               output='screen',
               arguments=['--ros-args', '--log-level', 'cam_pos:=DEBUG']),

          # ---- IK 逆解 -> 发布关节角 /joint_states ----
          Node(package='myik', executable='my_ik_node', name='my_ik_node',
               output='screen'),

    ])