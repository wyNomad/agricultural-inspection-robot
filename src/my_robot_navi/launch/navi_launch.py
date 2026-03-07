import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 获取路径
    pkg_navi = get_package_share_directory('my_robot_navi')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    # 指向你包内的数据库备份 (确保这个路径下真的有文件！)
    database_path = '/home/wyi/ros2_ws/src/my_robot_navi/maps/rtabmap1.db'
    # Nav2 参数文件路径
    nav2_params_file = os.path.join(pkg_navi, 'config', 'nav2_params.yaml')
    
    # 2. 构造关键参数字符串
    # 共用参数 (无论建图还是定位都需要)
    # -----------------------------------------------------------------------
    common_params = (
        '--ros-args '
        '-p subscribe_scan:= true '    # 开启雷达话题订阅
        '-p scan_topic:= /scan '     # 指定雷达话题名称
        
        '-p Reg/Strategy:= 2 '    # (Visual + ICP) (也可1(ICP))
        '-p Reg/Force3DoF:=true '           # 强制平面运动
        
        '-p Icp/VoxelSize:=0.05 '     #ICP 匹配前对点云进行下采样，提高计算速度
        '-p Icp/MaxCorrespondenceDistance:=0.1 '     # ICP 匹配的最大距离阈值，防止错误的关联
        '-p Icp/CorrespondenceRatio:=0.2 '    # 如果雷达扫描到的点匹配率低于 20%，则不采纳这次 ICP 结果，防止误匹配导致地图飞掉
        '-p Kp/MaxFeatures:=600 '            # 关键点特征上限

        '-p Vis/MaxFeatures:=1000 '          # 特征点上限
        '-p Vis/InlierDistance:=0.1 '        # 内点判断距离阈值
        '-p Vis/BundleAdjustment:=1 '         # 启用 BA 优化
        '-p RGBD/NeighborLinkRefining:=true ' # 空间一致性检查
        '-p RGBD/OptimizeFromGraphEnd:=true ' # 从图末端优化
        '-p RGBD/SavedLocalizationData:=true ' 
        '-p RGBD/ProximityBySpace:=true '    # 当机器人回到之前经过的地点（即使视觉没匹配上），也尝试用雷达探测闭环

        '-p Grid/FromDepth:=false '     # 将使用雷达数据（/scan）来构建 2D 占据栅格地图
        '-p Grid/RayTracing:=true '           # 启用光线追踪清除动态障碍
        '-p Grid/MaxGroundHeight:=0.15 '       # 障碍物高度阈值
        '-p Grid/RangeMax:=5 '              # 建图最大有效距离
        '-p Grid/NoiseFilteringRadius:=0.05 ' # 降噪
        '-p Grid/NoiseFilteringMinNeighbors:=8 '    # 提高邻居要求，强力去噪
        '-p Rtabmap/DetectionRate:=0.8 '      # 保护 CPU

        '-p approx_sync:=true '
        '-p sync_queue_size:=100 '
        '-p approx_sync_max_interval:=0.035 '
        '-p wait_for_transform:=0.2 '
    )

    # 建图模式专用 (Mapping)
    mapping_params = (
        '-d '                                  # 删除上一次数据
        '-p localization:=false '
        '-p Mem/IncrementalMemory:=true '      # 开启建图模式
        '-p RGBD/LinearUpdate:=0.1 '          # 移动 0.1m 更新一次
        '-p RGBD/AngularUpdate:=0.1 '         # 旋转 0.1rad 更新一次
    )
    # 定位模式专用 (Localization)
    localization_params = (
        '-p localization:=true '
        '-p Mem/IncrementalMemory:=false '    # 开启只读定位模式
        '-p Mem/InitWMWithAllNodes:=true '    # 加载全图加速重定位
        '-p Visual/MinInliers:=15 '           # 重定位门槛
        '-p RGBD/OptimizeMaxError:=2.0 '      # 放宽几何误差容忍度
        '-p RGBD/LoopClosureRecheck:=true '   # 失败后重新检查
        '-p RGBD/ProximityBySpace:=false '    # 重定位阶段关闭空间邻近限制
    )

    # 切换模式时，手动在这里修改：
    rtabmap_args_str = common_params + mapping_params       # 建图用这行
    # rtabmap_args_str = common_params + localization_params  # 定位用这行
    mode_is_localization = 'false'    # 建图就写false,定位就写true

    # 3. 包含官方启动文件
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={

            'localization': mode_is_localization,

            # 传入数据库路径
            'database_path': database_path,

            # --- 核心：注入我们的 参数字符串 ---
            'rtabmap_args': rtabmap_args_str,
            
            # --- 关键：接入 EKF，关闭自带 VO ---
            'visual_odometry': 'false',          # 必须关掉！
            'odom_topic': '/odometry/filtered',  # 连到 EKF 输出
            
            # --- 坐标系 ---
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',             # 确保和 EKF 一致
            
            # --- 传感器 ---
            'subscribe_depth': 'true',
            'approx_sync': 'true',
            'qos': '1', # 如果传感器数据不稳定，有时候改成 2 (Best Effort) 也有奇效
            
            # --- 话题重映射  ---
            'rgb_topic': '/camera/color/uncompressed',
            'depth_topic': '/camera/depth/uncompressedDepth',
            'camera_info_topic': '/camera/color/camera_info',
            
            # --- 其他 ---
            'use_sim_time': 'false',
            'rviz': 'true'

        }.items()
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # ===== 基础坐标系 =====
            "frame_id": "base_link",
            "odom_frame_id": "odom",

            # ===== 传感器订阅 =====
            "subscribe_rgbd": False,          # 不使用组合话题
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": True,
            "scan_topic": "/scan",

            # ===== 时间同步 =====
            "approx_sync": True,
            "sync_queue_size": 80,
            "topic_queue_size": 20,
            "approx_sync_max_interval": 0.035,

            # ===== TF 等待 =====
            "wait_for_transform": 0.3,

            "database_path": "/home/wyi/ros2_ws/src/my_robot_navi/maps/rtabmap.db",     # 地图数据库保存路径

            # ===== 定位模式关键参数 =====
            "Mem/IncrementalMemory": False,        # 不再增加新地图节点
            "RGBD/SavedLocalizationData": True,    # 使用已有地图，只定位，不建图

            # ===== ICP 点云配准（可选增强定位稳定性） =====
            "Reg/Strategy": 2,                     # 激光 + 视觉配准
            "Reg/Force3DoF": True,                 # 强制平面运动
            "Icp/VoxelSize": 0.05,
            "Icp/MaxCorrespondenceDistance": 0.1,
            "Icp/CorrespondenceRatio": 0.2,

            # ===== 视觉优化参数（可选） =====
            "RGBD/NeighborLinkRefining": True,
            "RGBD/OptimizeFromGraphEnd": True,
            "RGBD/ProximityBySpace": True,
            "Rtabmap/DetectionRate": 0.8,

            # ===== 2D 栅格地图（激光雷达辅助定位） =====
            "Grid/FromDepth": False,
            "Grid/RayTracing": True,
            "Grid/MaxGroundHeight": 0.15,
            "Grid/RangeMax": 5.0,
            "Grid/NoiseFilteringRadius": 0.05,
            "Grid/NoiseFilteringMinNeighbors": 8,
        }],
        remappings=[
            # 使用滤波后的 EKF 里程计
            ('odom', '/odometry/filtered'),
            # RGBD 话题
            ('rgb/image', '/camera/color/uncompressed'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            # 激光
            ('scan', '/scan'),
        ],
        # 定位模式不需要清空数据库，保留已有地图
        # arguments=['--delete_db_on_start']    # 建图模式用，定位模式应注释掉
    )



    nav2_launch_group = GroupAction(
        actions=[
            # --- 重映射开始 ---
            # 让 Nav2 听 EKF 的里程计
            SetRemap(src='/odom', dst='/odometry/filtered'),
            # 让 Nav2 听 RTAB-Map 的地图
            SetRemap(src='/map', dst='/rtabmap/map'),
            # (可选) 如果你的底盘话题不是 /cmd_vel，在这里取消注释并修改
            # SetRemap(src='/cmd_vel', dst='/robot/cmd_vel'),
            # --- 重映射结束 ---

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'autostart': 'true',  # 自动激活节点，不需要手动 lifecycle set
                    'use_composition': 'False', # 改为 False，让节点独立显示，方便调试
                    'container_name': 'nav2_container'
                }.items()
            )
        ]
    )

    # 4. 返回 LaunchDescription
    return LaunchDescription([
        rtabmap_node,
        # nav2_launch_group,
        # rtabmap_launch,
    ])