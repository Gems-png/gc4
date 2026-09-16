"""一键起 IK + FK + RViz，看机械臂而不是看 Qt 窗口。

    ros2 launch myfk arm_kinematics.launch.py
    ros2 launch myfk arm_kinematics.launch.py joint_topic:=/joint_states   # 没有下位机时干跑
    ros2 launch myfk arm_kinematics.launch.py rviz:=false                  # 只看节点不开界面

RViz 里能看到：
    机械臂骨架   /fk_arm_marker   （橙色连杆 + 蓝色关节球 + 黄色相机位置）
    抓取目标     /ik_target_marker（绿=可达，红=不可达）
    末端位姿     /fk_pose         （黄色坐标轴）
    相机位姿     /fk_camera_pose  （粉色坐标轴，末端上方 5cm）
固定坐标系固定用 base_link。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('myfk'), 'rviz', 'arm_kinematics.rviz')

    joint_topic = LaunchConfiguration('joint_topic')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_topic', default_value='/real_joint_states',
            description='myfk 听哪一路关节角：默认下位机回传的真实值；'
                        '没有下位机想干跑就填 /joint_states'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 rviz2'),

        # IK：目标点(mm) -> 关节角
        Node(package='myik', executable='my_ik_node', name='my_ik_node',
             output='screen'),

        # FK：关节角 -> 末端位姿 + RViz 里的机械臂
        Node(package='myfk', executable='myfk_node', name='myfk_node',
             parameters=[{'joint_topic': joint_topic}],
             output='screen'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz),
             output='screen'),
    ])
