#!/usr/bin/env python3
import math
import serial
import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# --------------------------------------------------------------------------
# 用户配置区域
# --------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyUSB0"  
BAUD_RATE = 115200

# --------------------------------------------------------------------------
# 协议常量
# --------------------------------------------------------------------------
CmdPacket_Begin = 0x49      # 起始码
CmdPacket_End = 0x4D        # 结束码
CmdPacketMaxDatSizeRx = 73  # 模块发来的数据包的数据体最大长度

class JackIM948(Node):
    
    def __init__(self):
        super().__init__("JackIM948")
        self.get_logger().info("正在初始化 IMU 节点...")

        # 1. 初始化串口
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"串口 {SERIAL_PORT} 打开成功")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            return

        # 2. 初始化协议变量
        self.CS = 0      # 校验和
        self.RxIndex = 0 # 接收索引
        self.buf = bytearray(5 + CmdPacketMaxDatSizeRx) 
        self.cmdLen = 0  
        self.i = 0       

        # 3. 创建发布者
        # 【确认】话题名称已改为 /imu/odom
        self.publisher_ = self.create_publisher(Imu, '/imu/odom', 10)

        # 4. 配置IMU
        self.configure_imu()

        # 5. 创建定时器
        # 【确认】频率设为 50Hz (1/50 = 0.02秒)
        self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("IMU 服务已就绪 (50Hz)，话题: /imu/odom")

    def configure_imu(self):
        """配置IMU硬件参数"""
        if not self.ser.is_open: return
        
        params = [0] * 11               
        isCompassOn = 0                 
        barometerFilter = 2             
        Cmd_ReportTag = 0x0FFF          
        
        params[0] = 0x12            
        params[1] = 5                   
        params[2] = 255                 
        params[3] = 0                   
        params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
        
        # 【确认】硬件输出频率设为 50Hz
        params[5] = 50                  
        
        params[6] = 1                   
        params[7] = 3                   
        params[8] = 5                   
        params[9] = Cmd_ReportTag & 0xff
        params[10] = (Cmd_ReportTag >> 8) & 0xff
        
        self.Cmd_PackAndTx(params, len(params)) 
        time.sleep(0.2)
        self.Cmd_PackAndTx([0x03], 1) # 唤醒
        time.sleep(0.2)
        self.Cmd_PackAndTx([0x19], 1) # 开启主动上报

    def timer_callback(self):
        """定时读取串口数据"""
        if not self.ser.is_open: return
        try:
            waiting_bytes = self.ser.in_waiting
            if waiting_bytes > 0:
                data = self.ser.read(waiting_bytes)
                for byte in data:
                    self.Cmd_GetPkt(byte)
        except Exception as e:
            self.get_logger().error(f"读取串口出错: {e}")

    # --------------------------------------------------------------------------
    # 核心发布函数 (已针对 EKF 优化)
    # --------------------------------------------------------------------------
    def pub(self, imu_data):
        imu_msg = Imu()
        # 1. 必填：时间戳和坐标系
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link' 

        # 2. 姿态 (Orientation) + 协方差
        if 'w' in imu_data:
            imu_msg.orientation.w = float(imu_data['w'])
            imu_msg.orientation.x = float(imu_data['x'])
            imu_msg.orientation.y = float(imu_data['y'])
            imu_msg.orientation.z = float(imu_data['z'])
            # EKF 信任度配置
            imu_msg.orientation_covariance = [
                1e-5, 0.0, 0.0,
                0.0, 1e-5, 0.0,
                0.0, 0.0, 1e-5
            ]

        # 3. 角速度 (Angular Velocity) + 协方差
        if 'angular_velocity_x' in imu_data:
            imu_msg.angular_velocity.x = float(imu_data['angular_velocity_x'])
            imu_msg.angular_velocity.y = float(imu_data['angular_velocity_y'])
            imu_msg.angular_velocity.z = float(imu_data['angular_velocity_z'])
            imu_msg.angular_velocity_covariance = [
                1e-4, 0.0, 0.0,
                0.0, 1e-4, 0.0,
                0.0, 0.0, 1e-4
            ]

        # 4. 线加速度 (Linear Acceleration) + 协方差
        if 'linear_acceleration_x' in imu_data:
            imu_msg.linear_acceleration.x = float(imu_data['linear_acceleration_x'])
            imu_msg.linear_acceleration.y = float(imu_data['linear_acceleration_y'])
            imu_msg.linear_acceleration.z = float(imu_data['linear_acceleration_z'])
            imu_msg.linear_acceleration_covariance = [
                5e-2, 0.0, 0.0,
                0.0, 5e-2, 0.0,
                0.0, 0.0, 5e-2
            ]
        
        self.publisher_.publish(imu_msg)

    # --------------------------------------------------------------------------
    # 协议解析部分
    # --------------------------------------------------------------------------
    def Cmd_PackAndTx(self, pDat, DLen):
        if DLen == 0 or DLen > 19: return -1 
        buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff,  0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])
        CS = sum(buf[51:51+DLen+2]) & 0xFF 
        buf.append(CS)
        buf.append(0x4D) 
        self.ser.write(buf)
        return 0

    def Cmd_GetPkt(self, byte):
        self.CS += byte 
        if self.RxIndex == 0: 
            if byte == CmdPacket_Begin:
                self.i = 0; self.buf[self.i] = CmdPacket_Begin; self.i += 1; self.CS = 0; self.RxIndex = 1
        elif self.RxIndex == 1: 
            self.buf[self.i] = byte; self.i += 1; self.RxIndex = (0 if byte == 255 else self.RxIndex + 1)
        elif self.RxIndex == 2: 
            self.buf[self.i] = byte; self.i += 1
            if byte > CmdPacketMaxDatSizeRx or byte == 0: self.RxIndex = 0
            else: self.RxIndex += 1; self.cmdLen = byte
        elif self.RxIndex == 3: 
            self.buf[self.i] = byte; self.i += 1
            if self.i >= self.cmdLen + 3: self.RxIndex += 1
        elif self.RxIndex == 4: 
            self.CS -= byte
            if (self.CS & 0xFF) == byte: self.buf[self.i] = byte; self.i += 1; self.RxIndex += 1
            else: self.RxIndex = 0
        elif self.RxIndex == 5: 
            self.RxIndex = 0
            if byte == CmdPacket_End: 
                self.buf[self.i] = byte; self.i += 1
                self.Cmd_RxUnpack(self.buf[3:self.i-2], self.i-5) 
                return 1
        else: self.RxIndex = 0
        return 0

    def Cmd_RxUnpack(self, buf, DLen):        
        # 【确认】保留原厂解析参数，绝不修改
        scaleAccel = 0.00478515625
        scaleQuat = 0.000030517578125
        scaleAngleSpeed = 0.06103515625
        
        if buf[0] == 0x11:
            ctl = (buf[2] << 8) | buf[1]
            imu_data = {}
            L = 7 
            
        # 1. 过滤掉 aX (0x0001)，保留原逻辑跳过
        if ((ctl & 0x0001) != 0): L += 6 

        # 2. 【关键修正】提取 AX, AY, AZ (0x0002) 作为带重力的加速度
        if ((ctl & 0x0002) != 0): 
                # 解析数据
            raw_ax = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel
            raw_ay = np.short((np.short(buf[L+3])<<8) | buf[L+2]) * scaleAccel
            raw_az = np.short((np.short(buf[L+5])<<8) | buf[L+4]) * scaleAccel
            imu_data['linear_acceleration_x'] = raw_ax
            imu_data['linear_acceleration_y'] = raw_ay
            imu_data['linear_acceleration_z'] = raw_az
            L += 6 

        # 3. 提取角速度 GX, GY, GZ (0x0004)
        if ((ctl & 0x0004) != 0):
            deg_to_rad = math.pi / 180.0
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed
            tmpY = np.short((np.short(buf[L+3])<<8) | buf[L+2]) * scaleAngleSpeed
            tmpZ = np.short((np.short(buf[L+5])<<8) | buf[L+4]) * scaleAngleSpeed
            imu_data['angular_velocity_x'] = tmpX * deg_to_rad
            imu_data['angular_velocity_y'] = tmpY * deg_to_rad
            imu_data['angular_velocity_z'] = tmpZ * deg_to_rad
            L += 6

        # 4. 过滤掉磁场 (0x0008)
        if ((ctl & 0x0008) != 0): L += 6
        # 5. 过滤掉温压高 (0x0010)
        if ((ctl & 0x0010) != 0): L += 8 

        # 6. 提取四元数 (0x0020)
        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat
            tmpX =   np.short((np.short(buf[L+3])<<8) | buf[L+2]) * scaleQuat
            tmpY =   np.short((np.short(buf[L+5])<<8) | buf[L+4]) * scaleQuat
            tmpZ =   np.short((np.short(buf[L+7])<<8) | buf[L+6]) * scaleQuat
            imu_data['w'] = tmpAbs
            imu_data['x'] = tmpX
            imu_data['y'] = tmpY
            imu_data['z'] = tmpZ
            L += 8

        # 7. 过滤掉其他不需要的数据，防止解析错位
        if ((ctl & 0x0040) != 0): L += 6 # Angle
        if ((ctl & 0x0080) != 0): L += 6 # Offset
        if ((ctl & 0x0100) != 0): L += 6 # Steps
        if ((ctl & 0x0200) != 0): L += 6 # asX (线性加速度去重力版，已跳过)
        if ((ctl & 0x0400) != 0): L += 2 # ADC
        if ((ctl & 0x0800) != 0): L += 1 # GPIO

        self.pub(imu_data)

def main():
    rclpy.init()
    node = JackIM948()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()