import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    
    # 1. 路径准备
    pkg_share = get_package_share_directory('my_robot_slam')
    
    # 定义配置文件夹路径 (解决 config_dir 未定义的问题)
    config_dir = os.path.join(pkg_share, 'config')

    # 定义具体文件路径
    rtabmap_config_path = os.path.join(config_dir, 'rtabmap_params.yaml')
    ekf_config_path = os.path.join(config_dir, 'efk.yaml')

    # 2. 声明开关
    # 定义启动参数，名字叫 'slam'，默认值 'False'
    slam_arg = DeclareLaunchArgument('slam',default_value='False',description='是否启动视觉SLAM节点')

    # 3. 图像解压节点：彩色图 (RGB)
    republish_rgb = Node(
        package='image_transport',
        executable='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/color/image_raw/compressed'),
            ('out', '/camera/color/uncompressed')
        ],
        output='log' # 关键：将所有输出(含Error)重定向到日志文件，保持终端干净
    )

    # 4. 图像解压节点：深度图 (Depth)
    republish_depth = Node(
        package='image_transport',
        executable='republish',
        arguments=['compressedDepth', 'raw'],
        remappings=[
            ('in/compressedDepth', '/camera/depth/image_raw/compressedDepth'),
            ('out', '/camera/depth/uncompressedDepth')
        ],
        output='log' # 这里的 Error 提示也会被隐藏
    )

    # 5.视觉slam
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[rtabmap_config_path],
        condition=IfCondition(LaunchConfiguration('slam')),
        remappings=[
            # [绝对控制] 这里写的什么，它就必须听什么！
            # 1. 强制订阅 EKF 的输出
            ('odom', '/odometry/filtered'),
            
            # 2. 强制订阅解压后的图像 (和 VO 节点一模一样)
            ('rgb/image',       '/camera/color/uncompressed'),
            ('depth/image',     '/camera/depth/uncompressedDepth'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            
            # 3. 可选：如果你要用 rtabmap_viz，可以把这个话题也对上
            # ('imu', '/imu/odom'), 
        ],
        arguments=['--delete_db_on_start'] # 调试期间每次清空地图
        
    )



    # 6.视觉里程计
    vision_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='visual_odometry',
        output='screen',
        parameters=[rtabmap_config_path],
            
        # [核心] 话题重映射
        remappings=[
            # 输入 (Input)
            ('rgb/image',       '/camera/color/uncompressed'),
            ('depth/image',     '/camera/depth/uncompressedDepth'),
            ('rgb/camera_info', '/camera/color/camera_info'),     
            # 输出 (Output) - 必须和 EKF 配置文件里的 odom1 名字一致
            ('odom',            '/vision/odom')
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 8.激光里程计
    # 获取 rf2o 的 launch 文件路径
    rf2o_launch_dir = os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch')
    # 调用 rf2o 的启动文件
    rf2o_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rf2o_launch_dir, 'rf2o_laser_odometry.launch.py')
            ),
    )
    # 启动你的协方差重写节点
    lidar_remapper = Node(
            package='my_robot_slam',
            executable='odom_remapper',
            name='lidar_odom_remapper',
    )

    # 9.efk里程计融合
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        # remappings=[('/odometry/filtered', '/odom')]       # 可选：如果你习惯订阅 /odom
    )
    
    return LaunchDescription([
        slam_arg,
        republish_rgb,
        republish_depth,
        slam_node,
        vision_node,
        rf2o_launch,
        lidar_remapper,
        ekf_node,
    ])