# 源码导览与历史环境参考

本文对应农业巡检机器人项目当前保留的早期 ROS 2 源码。用于查阅包结构、启动入口和历史配置；完整项目的运行环境仍待后续资料补充。

[返回项目介绍](../README.md)

## 功能与代码入口

| 模块 | 包名 | 内容 |
| --- | --- | --- |
| 机器人描述 | [`my_robot_description`](../src/my_robot_description) | 全向底盘 URDF、状态发布与 TF |
| 机械臂描述 | [`my_arm_description`](../src/my_arm_description) | 机械臂 URDF 与网格资源 |
| 运动规划 | [`my_arm_moveit_config`](../src/my_arm_moveit_config) | MoveIt 配置与演示入口 |
| RGB-D 目标感知 | [`my_robot_version`](../src/my_robot_version) | YOLO 检测、深度估计与目标三维位置 |
| 目标跟随 | [`my_robot_follow`](../src/my_robot_follow) | 跟随控制，输出 `/cmd_vel` |
| 建图与定位 | [`my_robot_slam`](../src/my_robot_slam) | RTAB-Map、RGB-D/ICP 里程计、EKF 配置 |
| 导航实验 | [`my_robot_navi`](../src/my_robot_navi) | RTAB-Map 定位与 Nav2 配置 |
| IMU / 相机 | [`imu_odometry`](../src/imu_odometry)、[`ros2_astra_camera`](../src/ros2_astra_camera) | IMU 数据与 Astra RGB-D 相机驱动 |

保留代码中的包名 `my_robot_version`；该包实际承担视觉感知功能。

## 环境与构建

需要 Linux、ROS 2、colcon 和 rosdep。下面以 ROS 2 Humble 的安装路径为例；完整环境版本与硬件组合尚未验证。

```bash
git clone https://github.com/wyNomad/agricultural-inspection-robot.git
cd agricultural-inspection-robot
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source install/setup.bash
```

按使用模块补齐依赖：

- 机械臂演示：MoveIt、`moveit_configs_utils`、控制器与 RViz。
- 建图/导航：RTAB-Map、Nav2、`robot_localization`、图像传输插件与相应传感器输入。
- 视觉检测：与当前 ROS Python 环境兼容的 Ultralytics、OpenCV、NumPy；仓库未锁定这些依赖版本。
- IMU：PySerial、NumPy，以及源码中配置的串口设备。
- Astra：参考[驱动原有说明](../src/ros2_astra_camera/README.MD)准备 SDK、系统依赖与设备配置。

`package.xml` 的依赖声明尚不完整，rosdep 成功不等于所有运行依赖已经齐备。

## 分模块运行

以下为独立入口，每个终端都需加载 ROS 2 和本工作空间环境。涉及传感器的节点还需准备对应硬件、话题与 TF。

| 任务 | 命令 |
| --- | --- |
| 发布底盘描述与 TF | `ros2 launch my_robot_description rsp_launch.py` |
| MoveIt 演示 | `ros2 launch my_arm_moveit_config demo.launch.py` |
| 三维目标检测 | `ros2 run my_robot_version object_detector_3d` |
| 目标跟随 | `ros2 run my_robot_follow follower_node` |
| 建图实验 | `ros2 launch my_robot_slam slam_launch.py` |
| 定位实验 | `ros2 launch my_robot_navi navi_launch.py` |

跟随与感知的默认话题不同：检测器发布 `/target/position_3d`，跟随节点订阅 `/target_point`。需要联调时，可在启动跟随节点时重映射：

```bash
ros2 run my_robot_follow follower_node --ros-args \
  -r /target_point:=/target/position_3d
```

重映射仅连接话题，还需确认目标坐标与控制算法的坐标约定一致。

## 运行前需要调整

- **地图路径**：SLAM 与定位启动文件仍引用原工作站的绝对路径。先修改为本机有效路径；本仓库当前树中没有对应的 `.db` 地图文件。
- **地图写入行为**：`slam_launch.py` 设置了 `--delete_db_on_start`，启动时会清空其配置路径指向的旧数据库。使用独立的建图输出路径。
- **导航启动范围**：`navi_launch.py` 当前返回的启动项只有 RTAB-Map 节点；`nav2_launch_group` 被注释，运行该文件不会同时启动完整 Nav2。
- **输入与 TF**：检查 RGB、深度、相机内参、`/scan`、里程计和坐标树，图像压缩与解压话题也需要对齐。
- **IMU 设备**：串口在 `imu_node.py` 中配置，尚未统一为 launch 参数。
- **模型资源**：视觉包含 `.pt` / `.onnx` 文件，默认模型是 `yolov8n.pt`。自定义模型的类别、输入格式和来源需要单独核对。

## 工作空间加载

使用本项目时单独构建并加载当前工作空间；其他工作空间中若存在同名 ROS 包，应避免同时叠加，以免加载到不同版本的配置。本项目作为独立的农业巡检机器人项目保留。

## 验证与后续维护

本轮整理核对了包清单、注册的可执行程序、启动文件与配置。尚未完成全量构建、传感器联调或机械臂实机验证。

后续重点：记录可复现环境、参数化地图/串口、明确 TF 与话题接口，补充完整版本的环境与运行记录。

## 代码来源与许可证

本仓库包含第三方驱动与模型资源。各子目录的许可证及来源说明分别适用，不能将整个工作空间直接视为统一授权的原创代码。分享或再分发前请核对对应资源的授权。
