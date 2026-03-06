import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time
from ament_index_python.packages import get_package_share_directory

# === 状态定义 ===
STATUS_SEARCHING = 0  # 寻找目标中
STATUS_LOCKED = 1     # 已锁定目标
STATUS_LOST_BUFFER = 2 # 目标短暂丢失（给一点时间等待重连）

class ObjectDetector3D(Node):
    def __init__(self):
        super().__init__('object_detector_3d')

        # === 1. 参数配置 ===
        self.declare_parameter('model_name', 'yolov8n.pt') # 或 best.onnx
        self.declare_parameter('target_class', 'bottle')   # 你想追踪的类别
        self.declare_parameter('conf_thres', 0.15)         # 置信度
        self.declare_parameter('lost_timeout', 1.0)        # 丢失多少秒后重置
        
        # 获取参数
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.lost_timeout = self.get_parameter('lost_timeout').get_parameter_value().double_value

        # === 2. 加载模型 ===
        # 自动寻找 share 目录下的 config 文件夹
        try:
            # 如果是标准安装路径
            pkg_share = get_package_share_directory('my_robot_version')
            model_path = os.path.join(pkg_share, 'config', model_name)
        except:
            # 如果是直接运行脚本（调试用）
            model_path = model_name 
            
        self.get_logger().info(f"正在加载模型: {model_path} | 目标类别: {self.target_class}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            return

        # === 3. 初始化状态机变量 ===
        self.state = STATUS_SEARCHING
        self.locked_id = -1          # 当前锁定的 ID
        self.last_seen_time = 0.0    # 上次看到目标的时间戳
        self.tracking_failures = 0   # 连续丢失帧数计数

        # === 4. ROS 通信初始化 ===
        self.bridge = CvBridge()
        self.intrinsics = None
        
        # 订阅内参
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)

        # 时间同步订阅 RGB 和 Depth
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        # 允许 0.1s 误差
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.target_pub = self.create_publisher(PointStamped, '/target/position_3d', 10)
        self.get_logger().info("✅ 工业级追踪系统启动完毕")

    def info_cb(self, msg):
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = True
            self.get_logger().info(f"📸 内参锁定: {msg.width}x{msg.height}")

    def get_robust_depth(self, cv_depth, center_u, center_v):
        """
        工业级深度提取：
        不只取一个点，而是取中心 5x5 区域的中值，剔除 0 值（无效值）
        """
        height, width = cv_depth.shape
        roi_size = 5 # 5x5 区域
        half = roi_size // 2
        
        # 边界检查
        u_min = max(0, center_u - half)
        u_max = min(width, center_u + half + 1)
        v_min = max(0, center_v - half)
        v_max = min(height, center_v + half + 1)
        
        roi = cv_depth[v_min:v_max, u_min:u_max]
        
        # 剔除 0 值 (0 代表深度相机没测出来)
        valid_pixels = roi[roi > 0]
        
        if len(valid_pixels) == 0:
            return 0.0 # 真的没深度
        
        # 取中值 (比平均值更抗噪)
        return np.median(valid_pixels)

    def sync_callback(self, color_msg, depth_msg):
        if not self.intrinsics: return

        try:
            # 1. 图像转换
            cv_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            current_ros_time = self.get_clock().now().nanoseconds / 1e9

            # 2. YOLO 追踪推理 (ByteTrack)
            results = self.model.track(cv_image, persist=True, tracker="bytetrack.yaml", 
                           conf=self.conf_thres, verbose=False, task='detect')[0]

            # 3. 整理有效目标列表
            valid_targets = []
            
            for box in results.boxes:
                # 获取基本信息
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                
                # 过滤：必须是我们要找的类别 (比如 'bottle')
                if class_name != self.target_class:
                    continue
                
                # 获取 ID (如果没有 ID，说明还没跟稳，暂时设为 -1)
                track_id = int(box.id[0].item()) if box.id is not None else -1
                
                # 获取坐标
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_u = int((x1 + x2) / 2)
                center_v = int((y1 + y2) / 2)
                
                # 计算与画面中心的偏差 (用于择优)
                img_cx = cv_image.shape[1] // 2
                offset_x = abs(center_u - img_cx)
                
                valid_targets.append({
                    'id': track_id,
                    'box': (x1, y1, x2, y2),
                    'center': (center_u, center_v),
                    'offset': offset_x
                })

            # === 4. 状态机核心逻辑 ===
            
            target_to_publish = None # 最终决定要发布的目标

            # --- 状态 A: SEARCHING (寻找模式) ---
            if self.state == STATUS_SEARCHING:
                if len(valid_targets) > 0:
                    # 策略：选择离画面中心最近的一个目标进行锁定
                    # 按照 offset 排序，取最小的
                    best_target = min(valid_targets, key=lambda t: t['offset'])
                    
                    if best_target['id'] != -1:
                        self.state = STATUS_LOCKED
                        self.locked_id = best_target['id']
                        self.last_seen_time = current_ros_time
                        self.get_logger().info(f"🔐 锁定目标 ID: {self.locked_id}")
                        target_to_publish = best_target
                    else:
                        # 虽然检测到了但还没 ID，先临时发一下，但不锁定
                        target_to_publish = best_target

            # --- 状态 B: LOCKED (锁定模式) ---
            elif self.state == STATUS_LOCKED:
                found_it = False
                for t in valid_targets:
                    if t['id'] == self.locked_id:
                        # 找到了我们的老朋友！
                        self.last_seen_time = current_ros_time
                        target_to_publish = t
                        found_it = True
                        break
                
                if not found_it:
                    # 这一帧没看到锁定的目标 -> 进入缓冲检查
                    # 检查是不是超时了
                    if current_ros_time - self.last_seen_time > self.lost_timeout:
                        self.get_logger().warn(f"❌ 目标 ID:{self.locked_id} 丢失太久，重置为搜索模式")
                        self.state = STATUS_SEARCHING
                        self.locked_id = -1
                    else:
                        self.state = STATUS_LOST_BUFFER
                        # 在 BUFFER 状态下，我们不发布新坐标，保持静默，等待重连
                        
            # --- 状态 C: LOST BUFFER (缓冲模式) ---
            elif self.state == STATUS_LOST_BUFFER:
                # 逻辑同 LOCKED，如果在 valid_targets 里找回了 ID，就切回 LOCKED
                found_it = False
                for t in valid_targets:
                    if t['id'] == self.locked_id:
                        self.state = STATUS_LOCKED
                        self.last_seen_time = current_ros_time
                        target_to_publish = t
                        self.get_logger().info(f"🔄 目标 ID:{self.locked_id} 重连成功！")
                        found_it = True
                        break

                if not found_it:
                    if current_ros_time - self.last_seen_time > self.lost_timeout:
                        self.state = STATUS_SEARCHING
                        self.locked_id = -1

            # === 5. 数据处理与发布 ===
            if target_to_publish:
                u, v = target_to_publish['center']
                
                # 鲁棒深度获取 (取 ROI 中值)
                d_raw = self.get_robust_depth(cv_depth, u, v)
                
                if d_raw > 0:
                    z_metric = d_raw / 1000.0
                    x_metric = (u - self.cx) * z_metric / self.fx
                    y_metric = (v - self.cy) * z_metric / self.fy
                    
                    # 发布
                    point_msg = PointStamped()
                    point_msg.header.stamp = color_msg.header.stamp
                    point_msg.header.frame_id = "camera_color_optical_frame"
                    point_msg.point.x = x_metric
                    point_msg.point.y = y_metric
                    point_msg.point.z = z_metric
                    self.target_pub.publish(point_msg)

                    # 可视化用的数据
                    viz_x, viz_y, viz_z = x_metric, y_metric, z_metric
                else:
                    viz_x, viz_y, viz_z = 0, 0, 0 # 无效深度

            # === 6. 工业级可视化 UI ===
            # 绘制所有检测框(灰色)，锁定的框(绿色)，丢失中的框(红色虚影-这里简化为不画)
            for t in valid_targets:
                x1, y1, x2, y2 = t['box']
                if t['id'] == self.locked_id:
                    color = (0, 255, 0) # 绿色：锁定
                    thickness = 3
                else:
                    color = (200, 200, 200) # 灰色：路人
                    thickness = 1
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, thickness)
                cv2.putText(cv_image, f"ID:{t['id']}", (x1, y1-10), 0, 0.6, color, 2)

            # 屏幕左上角显示系统状态
            status_text = "SEARCHING"
            status_color = (0, 165, 255) # 橙色
            if self.state == STATUS_LOCKED:
                status_text = f"LOCKED: {self.locked_id}"
                status_color = (0, 255, 0) # 绿色
            elif self.state == STATUS_LOST_BUFFER:
                status_text = f"LOST (Waiting...): {self.locked_id}"
                status_color = (0, 0, 255) # 红色

            cv2.putText(cv_image, f"MODE: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            # 显示图像
            cv2.imshow("Industrial Tracker", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = ObjectDetector3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()