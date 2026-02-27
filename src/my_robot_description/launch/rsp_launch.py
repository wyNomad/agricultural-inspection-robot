import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot_description'
    urdf_name = "omni_car.urdf"


    # 获取 URDF 路径
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. 机器人状态发布器 (核心)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 简单的逻辑：我们直接加载普通的 joint_state_publisher
    # 它可以自动发布默认位置，让轮子显示出来，且不弹窗
    # (如果你以后真的需要弹窗，可以临时把 executable 改回 'joint_state_publisher_gui')
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # parameters=[{'source_list': ['/wheel/speed']}] # 可选配置
    )

    # 将雷达所用的基坐标接入TF树

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 参数顺序：x y z yaw pitch roll frame_id child_frame_id
        arguments = ['0', '0', '0', '0', '0', '0', 'laser_link', 'laser']
    )

    return LaunchDescription([
        jsp_node,
        robot_state_publisher_node,
        lidar_tf,
    ])