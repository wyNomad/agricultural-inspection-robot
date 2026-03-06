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
    slam_arg = DeclareLaunchArgument('slam',default_value='True',description='是否启动视觉SLAM节点')

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

    # 5.启动slam
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # ===== 基础坐标系 =====
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            
            # ===== 传感器订阅 =====
            "subscribe_rgbd": False,          # 不使用 RGB-D 组合话题，分别订阅
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": True,
            
            # ===== 时间同步 =====
            "approx_sync": True,                # 启用近似时间同步
            "sync_queue_size": 80,                    # 同步队列大小（兼顾 CPU 与防丢帧，可根据传感器频率调整至 80~100
            "topic_queue_size": 20,
            "approx_sync_max_interval": 0.035,   # 最大允许时间差（秒），保证数据对齐精度
            
            # # ===== TF 等待 =====
            "wait_for_transform": 0.3,           # 等待 TF 变换的超时时间（秒）
            
            # # ===== 数据库 =====
            # "database_path": "~/rtabmap.db",     # 地图数据库保存路径
            
            # # ===== 建图模式 =====
            "Mem/IncrementalMemory": "True",       # 增量式建图
            "Mem/InitWMWithAllNodes": "False",     # 不一开始加载所有节点
            
            # # ===== 视觉特征参数 =====
            "Vis/MinInliers": "20",                 # 视觉里程计最小内点数
            "Vis/MaxFeatures": "1000",               # 特征点数量上限
            "Vis/InlierDistance": "0.1",             # 内点重投影误差阈值（单位：像素或米，需查文档，通常 0.1 较严格）
            "Vis/BundleAdjustment": "1",              # 启用局部光束法平差（1=启用）
            "Kp/MaxFeatures": "600",                  # 每帧图像提取的关键点上限
            
            # # ===== 激光-视觉融合配准 =====
            "Reg/Strategy": "2",                      # 配准策略：2=视觉 + ICP
            "Reg/Force3DoF": "True",                   # 强制平面运动（适用于轮式机器人）
            
            # # ===== ICP 点云配准 =====
            "Icp/VoxelSize": "0.05",                   # 下采样体素大小（米）
            "Icp/MaxCorrespondenceDistance": "0.1",    # 最大对应距离（米），防止错误关联
            "Icp/CorrespondenceRatio": "0.2",           # 最小匹配率（低于此值则丢弃该帧）
            
            # # ===== 里程计与闭环优化 =====
            "RGBD/NeighborLinkRefining": "True",        # 空间一致性检查（剔除错误闭环）
            "RGBD/OptimizeFromGraphEnd": "True",        # 从图末端优化，提高局部一致性
            "RGBD/SavedLocalizationData": "True",       # 保存定位数据，便于后续纯定位模式
            "RGBD/ProximityBySpace": "True",             # 根据空间距离触发闭环检测（即使视觉没匹配）
            "Rtabmap/DetectionRate": "0.8",              # 闭环检测频率（Hz），保护 CPU
            
            # # ===== 2D 占据栅格地图（激光雷达） =====
            "Grid/FromDepth": "False",                   # 使用激光数据而非深度图像建图
            "Grid/RayTracing": "True",                    # 光线追踪，清除动态障碍
            "Grid/MaxGroundHeight": "0.15",                # 障碍物高度阈值（米），高于此视为障碍
            "Grid/RangeMax": "5.0",                        # 激光最大有效建图距离（米）
            "Grid/NoiseFilteringRadius": "0.05",           # 降噪半径（米）
            "Grid/NoiseFilteringMinNeighbors": "8",        # 降噪所需最少邻居数（强力去噪，可根据环境适当降低至 5~6）
        }],
        remappings=[
            ('odom', '/odometry/filtered'),               # 使用滤波后的里程计话题
            ('rgb/image', '/camera/color/uncompressed'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan', '/scan'),
        ],
        arguments=['--delete_db_on_start']                # 每次启动清空旧数据库，适合建图阶段
    )

    # 6.视觉里程计
    vision_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='visual_odometry',
        output='screen',
        parameters=[
            rtabmap_config_path,  # 原有配置文件
            # 新增：解决RGB/深度时间戳同步警告的参数
            {'approx_sync_max_interval': 0.03},  # 放宽同步阈值到30毫秒（推荐新手先用这个）
            # {'approx_sync': False}  # 备选：严格要求时间戳完全一致（如果相机支持硬件同步再用）
        ],
        # [核心] 话题重映射
        remappings=[
            # 输入 (Input)
            ('rgb/image',       '/camera/color/uncompressed'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),     
            # 输出 (Output) - 必须和 EKF 配置文件里的 odom1 名字一致
            ('odom',            '/vision/odom')
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 7.ICP里程计节点配置
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[
            {'frame_id': 'base_link'},          # 激光雷达所在坐标系
            {'odom_frame_id': 'odom'},          # 里程计输出的父坐标系
            {'publish_tf': False},               # 是否发布tf变换
            {'approx_sync': True}               # 启用近似同步
        ],
        remappings=[
            ('scan', '/scan'),  # 激光雷达扫描数据话题重映射
            ('odom', '/lidar/odom')
        ],
    )

    # 8.efk里程计融合
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
        icp_odometry_node,
        vision_node,
        ekf_node,
    ])
