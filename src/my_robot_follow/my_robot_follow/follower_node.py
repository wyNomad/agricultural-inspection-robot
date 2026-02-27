import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math
import time

# --- 工业级 PID 工具类 ---
class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out, dead_zone=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out  # 输出下限 (比如 -0.5 m/s)
        self.max_out = max_out  # 输出上限 (比如 0.5 m/s)
        self.dead_zone = dead_zone # 死区阈值
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        current_time = time.time()
        
        # 0. 初始化时间
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 防止 dt 为 0 或过大 (系统卡顿处理)
        if dt <= 0 or dt > 0.5: 
            return 0.0

        # 1. 死区处理 (Dead Zone) - 在目标附近就“装死”，防止抖动
        if abs(error) < self.dead_zone:
            self.prev_error = 0.0
            self.integral = 0.0
            return 0.0

        # 2. 积分计算 (带积分限幅 Anti-windup)
        # 只有在误差较小时才积分，防止由于大误差导致的积分饱和
        if abs(error) < 0.5: 
            self.integral += error * dt
        # 简单的积分限幅，防止由于积分过大导致超调
        self.integral = max(min(self.integral, 1.0), -1.0) 

        # 3. 微分计算
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # 4. PID 输出
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # 5. 输出硬限幅 (Saturation)
        output = max(min(output, self.max_out), self.min_out)
        
        return output

# --- 平滑滤波器 (防止起步/刹车太猛) ---
class RampFilter:
    def __init__(self, max_accel):
        self.max_accel = max_accel # 最大加速度 unit/s^2
        self.last_output = 0.0
        self.last_time = time.time()

    def filter(self, target_output):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.5: return 0.0

        # 计算这一帧允许的最大变化量
        max_delta = self.max_accel * dt
        
        delta = target_output - self.last_output
        
        # 限制变化量
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
            
        self.last_output += delta
        return self.last_output

# --- ROS 2 节点 ---
class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # === 1. 参数声明 (方便在 launch 文件里调参) ===
        # 距离 PID (控制前后)
        self.declare_parameter('dist_kp', 0.8)
        self.declare_parameter('dist_kd', 0.05)
        self.declare_parameter('target_dist', 0.8) # 目标保持 0.8米
        
        # 角度 PID (控制旋转)
        self.declare_parameter('angle_kp', 1.5)
        self.declare_parameter('angle_kd', 0.1)
        
        # 限制
        self.declare_parameter('max_linear_speed', 0.4) # 最大线速度 m/s
        self.declare_parameter('max_angular_speed', 1.0) # 最大角速度 rad/s
        self.declare_parameter('max_linear_accel', 0.5) # 线加速度 m/s^2 (防止打滑)

        # === 2. 初始化控制器 ===
        # 获取参数
        dist_kp = self.get_parameter('dist_kp').value
        dist_kd = self.get_parameter('dist_kd').value
        self.target_dist = self.get_parameter('target_dist').value
        
        angle_kp = self.get_parameter('angle_kp').value
        angle_kd = self.get_parameter('angle_kd').value
        
        max_v = self.get_parameter('max_linear_speed').value
        max_w = self.get_parameter('max_angular_speed').value
        max_acc = self.get_parameter('max_linear_accel').value

        # 实例化 PID (距离控制只用 PD，死区 0.05m)
        self.pid_dist = PIDController(dist_kp, 0.0, dist_kd, -max_v, max_v, dead_zone=0.05)
        
        # 实例化 PID (角度控制只用 PD，死区 0.05 rad 约3度)
        self.pid_angle = PIDController(angle_kp, 0.0, angle_kd, -max_w, max_w, dead_zone=0.05)
        
        # 实例化平滑器
        self.ramp_linear = RampFilter(max_accel=max_acc)

        # === 3. 通信接口 ===
        # 订阅视觉
        self.sub = self.create_subscription(PointStamped, '/target_point', self.vision_callback, 10)
        # 发布速度
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # === 4. 安全看门狗 (Watchdog) ===
        # 记录最后收到消息的时间，防止视觉丢帧后机器人疯跑
        self.last_msg_time = time.time()
        self.create_timer(0.1, self.watchdog_callback) # 10Hz 检查一次

        self.get_logger().info('工业级跟随节点已启动。Waiting for target...')

    def vision_callback(self, msg):
        self.last_msg_time = time.time()
        
        # --- 坐标系映射 (非常重要!) ---
        # 假设相机系: Z向前(深度), X向右, Y向下
        # 底盘系: Linear.x 向前, Angular.z 旋转
        
        # 1. 提取原始数据
        cam_x = msg.point.x # 横向偏差
        cam_z = msg.point.z # 纵向深度 (距离)
        
        # 2. 计算误差
        # 距离误差: 目标 > 设定值 -> error > 0 -> 向前跑
        error_dist = cam_z - self.target_dist
        
        # 角度误差: 使用反正切计算具体角度 (rad)
        # 如果物体在右边 (cam_x > 0)，我们需要右转 (angular.z < 0)
        # 所以 error_angle 应该是 0 - 当前角度
        target_angle = math.atan2(cam_x, cam_z) # 简单的几何关系
        error_angle = 0.0 - target_angle

        # 3. 计算 PID 输出
        raw_linear = self.pid_dist.compute(error_dist)
        raw_angular = self.pid_angle.compute(error_angle)
        
        # 4. 速度平滑处理 (仅对线速度做平滑，转向通常需要响应快)
        smooth_linear = self.ramp_linear.filter(raw_linear)

        # 5. 发布
        cmd = Twist()
        cmd.linear.x = smooth_linear
        cmd.angular.z = raw_angular
        self.pub.publish(cmd)

    def watchdog_callback(self):
        # 如果超过 0.5 秒没有收到视觉消息，强制停车
        if time.time() - self.last_msg_time > 0.5:
            stop_cmd = Twist() # 默认全是 0
            self.pub.publish(stop_cmd)
            # 只有第一次超时打印一次警告，防止刷屏
            # self.get_logger().warn('视觉数据丢失，安全停车中...', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一个停车指令
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 