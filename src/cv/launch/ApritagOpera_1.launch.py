from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 使用示例：
# 真实摄像头 + 真实串口（需指定串口设备）
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=false serial_sim:=false port:=/dev/ttyUSB0
# 真实摄像头 + 串口模拟（无物理串口）
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=false serial_sim:=true
# 模拟图像（合成 tag）+ 串口模拟
# ros2 launch cv ApritagOpera_1.launch.py use_sim:=true serial_sim:=true
# 只跑 tag 检测测距（不开 IK/串口）：serial_sim 对 cam_pos 无影响，IK 节点始终会启动

# 相机的设备名/分辨率/帧率**不从这里传**：那是 Minit.open_cap 的形参，
# 默认值在同文件的 DEFAULT_SIZE / DEFAULT_FPS，要换相机/换分辨率改那儿。
# 看设备现状用 ls /dev/video*（注意设备号会随插拔漂），调试工具 guvcview。

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='true')
    serial_sim = LaunchConfiguration('serial_sim', default='true')

    # --- 串口端口 ---
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    # 杆长不再从 launch 传：myik / myfk 节点里的默认值就是实测值，保持单一来源，
    # 避免出现 launch 写 300/300、代码里是 205.23 这种两边对不上的情况。

    # tag 边长 / id / 内参文件也**不从这里传**：归 apriltag 模块 (AprilTagParams),
    # 标定文件它自己找 (share/cv/config/gc480p.json)。这里只挑用哪种位姿算法。

    return LaunchDescription([
          DeclareLaunchArgument('use_sim', default_value='true',
                                   description='true=合成tag图像(tag_image_pub), false=真摄像头(raw_image_pub)'),
          DeclareLaunchArgument('serial_sim', default_value='true',
                                   description='true=串口sim模式(无设备也能跑, 仅打印帧), false=真实串口'),
          DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                                   description='串口设备路径（仅 serial_sim:=false 时有效）'),

          # ---- 图像来源 + tag 位姿: 收敛为 MainCam 单节点 ----
          # use_sim:=true  -> 发布合成 AprilTag 图像 (原 tag_image_pub 功能)
          # use_sim:=false -> 直驱真实摄像头 (原 raw_image_pub / usb_cam 功能)
          # pose_method 不写 = 用 apriltag 模块的默认 (pnp); 本文件要的是相似三角形
          # 测距 (原 cam_pos), 所以这一个覆盖掉。
          Node(package='cv', executable='main_cam', name='main_cam',
               parameters=[{
                    'use_sim': use_sim,
                    'frame_id': 'camera_frame',
                    'pose_method': 'similar',
               }],
               output='screen'),

          # ---- IK 逆解 -> 发布关节角 /joint_states ----
          Node(package='myik', executable='my_ik_node', name='my_ik_node',
               output='screen'),

    ])