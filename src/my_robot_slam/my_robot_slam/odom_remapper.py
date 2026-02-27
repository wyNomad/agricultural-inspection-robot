import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovarianceRemapper(Node):
    def __init__(self):
        super().__init__('odom_covariance_remapper')
        # 订阅 rf2o 发布的原始话题
        self.sub = self.create_subscription(Odometry, '/rf2o/lidar/odom', self.callback, 10)
        # 发布给 EKF 的修正后话题
        self.pub = self.create_publisher(Odometry, '/lidar/odom', 15)

    def callback(self, msg):
        # --- 1. 修改位姿协方差 (Pose Covariance) ---
        # 即使 rf2o 原本有值，我们也强制覆盖为我们设定的权重
        p_cov = [0.0] * 36
        p_cov[0] = 5e-5  # X 的信任度
        p_cov[7] = 5e-5  # Y 的信任度
        p_cov[35] = 1e-3 # Yaw 的信任度
        msg.pose.covariance = p_cov
        # --- 2. 修改速度协方差 (Twist Covariance) ---
        # 如果这里全是0，EKF 会认为速度绝对准，建议也给一点噪声值
        t_cov = [0.0] * 36
        t_cov[0]  = 5e-3  # 线速度 Vx
        t_cov[7]  = 5e-3  # 线速度 Vy
        t_cov[35] = 5e-3  # 角速度 V-yaw
        msg.twist.covariance = t_cov

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomCovarianceRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()