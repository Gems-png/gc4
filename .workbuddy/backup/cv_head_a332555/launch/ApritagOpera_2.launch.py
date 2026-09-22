import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 使用示例：
# 真实摄像头 + 真实串口（需指定串口设备）
# ros2 launch cv ApritagOpera_2.launch.py use_sim:=false serial_sim:=false port:=/dev/ttyUSB0 camera_id:=4 freq:=15.0
# 真实摄像头 + 串口模拟（无物理串口）
# ros2 launch cv ApritagOpera_2.launch.py use_sim:=false serial_sim:=true
# 模拟图像（合成 tag）+ 串口模拟
# ros2 launch cv ApritagOpera_2.launch.py use_sim:=true serial_sim:=true

# sudo apt install guvcview
# guvcview -d /dev/video4

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='true')
    serial_sim = LaunchConfiguration('serial_sim', default='true')
    
    # --- 新增：摄像头参数 ---
    camera_id = LaunchConfiguration('camera_id')      # 设备号（对应 /dev/video4）
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    freq = LaunchConfiguration('freq', default='15.0')            # 发布帧率
    
    # --- 新增：串口端口 ---
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    
    # 杆长不再从 launch 传：myik / myfk 节点里的默认值就是实测值，保持单一来源，
    # 避免出现 launch 写 300/300、代码里是 205.23 这种两边对不上的情况。

    calib_file = os.path.join(
        get_package_share_directory('cv'), 'config', 'gc480p.json')

    return LaunchDescription([
          DeclareLaunchArgument('use_sim', default_value='true',
                                   description='true=合成tag图像(tag_image_pub), false=真摄像头(raw_image_pub)'),
          DeclareLaunchArgument('serial_sim', default_value='true',
                                   description='true=串口sim模式(无设备也能跑, 仅打印帧), false=真实串口'),
          DeclareLaunchArgument('camera_id', default_value='4',
                                   description='摄像头设备号（对应 /dev/video 后的数字）'),
          DeclareLaunchArgument('width', default_value='640',
                                   description='图像宽度（像素）'),
          DeclareLaunchArgument('height', default_value='480',
                                   description='图像高度（像素）'),
          DeclareLaunchArgument('freq', default_value='15.0',
                                   description='发布图像的目标帧率（Hz）'),
          DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                                   description='串口设备路径（仅 serial_sim:=false 时有效）'),

          # ---- 图像来源 ----
          Node(package='cv', executable='tag_image_pub', name='tag_image_pub',
               condition=IfCondition(PythonExpression(["'", use_sim, "' == 'true'"])),
               output='screen'),
          Node(package='cv', executable='raw_image_pub', name='raw_image_pub',
               condition=IfCondition(PythonExpression(["'", use_sim, "' == 'false'"])),
               parameters=[{
                    'camera_id': camera_id,
                    'width': width,
                    'height': height,
                    'freq': freq,
                    # 'frame_id' 可保持默认 'camera_frame'，如果需要也可添加参数
               }],
               output='screen'),

          # ---- tag 检测 -> 发布目标坐标 /goal_position ----
          Node(package='cv', executable='tag_pose', name='tag_pose',
               parameters=[{'calib_file': calib_file}],
               output='screen',
               # ↓↓↓ 添加这一行，仅提升该节点的日志级别 ↓↓↓
               arguments=['--ros-args', '--log-level', 'tag_pose:=DEBUG']
          ),

          # ---- IK 逆解 -> 发布关节角 /joint_states ----
          Node(package='myik', executable='my_ik_node', name='my_ik_node',
               output='screen'),

          # ---- 串口：driver 管字节，bridge 管协议 ----
          Node(package='serial_comm', executable='serial_driver', name='serial_driver',
               parameters=[{
                    'simulate': serial_sim,
                    'port': port,          # 将端口参数传入，节点内部需要支持此参数
               }],
               output='screen'),
          Node(package='serial_comm', executable='serial_bridge', name='serial_bridge',
               output='screen'),
    ])