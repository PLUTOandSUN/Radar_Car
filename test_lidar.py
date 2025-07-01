#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
思岚C1雷达测试脚本
用于测试STM32雷达车的雷达控制和数据接收功能
"""

import serial
import struct
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading

# 通信协议常量
FRAME_HEADER = 0x55AA
FRAME_TAIL = 0x0D0A
DEVICE_STM32 = 0x01
DEVICE_PC = 0x04

# 命令类型
CMD_LIDAR_START_SCAN = 0x10
CMD_LIDAR_STOP_SCAN = 0x11
CMD_LIDAR_START_MOTOR = 0x12
CMD_LIDAR_STOP_MOTOR = 0x13
CMD_LIDAR_RESET = 0x14
CMD_LIDAR_GET_STATUS = 0x15
CMD_LIDAR_DATA = 0x04
CMD_SYSTEM_INFO = 0x08
CMD_ACK = 0x0A

class LidarController:
    def __init__(self, port='COM3', baudrate=9600):
        """初始化雷达控制器"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"串口已连接: {port} @ {baudrate}")
        except Exception as e:
            print(f"串口连接失败: {e}")
            self.ser = None
        
        self.scan_data = []
        self.is_receiving = False
        self.lock = threading.Lock()
        
    def calc_crc16(self, data):
        """计算CRC16校验值"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc
    
    def pack_frame(self, device_id, cmd_type, data=b''):
        """打包通信帧"""
        frame = bytearray()
        
        # 帧头
        frame.extend(struct.pack('>H', FRAME_HEADER))
        
        # 设备ID
        frame.append(device_id)
        
        # 命令类型
        frame.append(cmd_type)
        
        # 数据长度
        frame.append(len(data))
        
        # 数据
        frame.extend(data)
        
        # 计算CRC
        crc_data = frame[2:]
        crc = self.calc_crc16(crc_data)
        frame.extend(struct.pack('>H', crc))
        
        # 帧尾
        frame.extend(struct.pack('>H', FRAME_TAIL))
        
        return frame
    
    def send_command(self, cmd_type, data=b''):
        """发送命令"""
        if not self.ser:
            return False
        
        frame = self.pack_frame(DEVICE_PC, cmd_type, data)
        try:
            self.ser.write(frame)
            print(f"发送命令: 0x{cmd_type:02X}")
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False
    
    def start_motor(self):
        """启动雷达电机"""
        return self.send_command(CMD_LIDAR_START_MOTOR)
    
    def stop_motor(self):
        """停止雷达电机"""
        return self.send_command(CMD_LIDAR_STOP_MOTOR)
    
    def start_scan(self):
        """开始扫描"""
        return self.send_command(CMD_LIDAR_START_SCAN)
    
    def stop_scan(self):
        """停止扫描"""
        return self.send_command(CMD_LIDAR_STOP_SCAN)
    
    def reset_lidar(self):
        """复位雷达"""
        return self.send_command(CMD_LIDAR_RESET)
    
    def get_status(self):
        """获取雷达状态"""
        return self.send_command(CMD_LIDAR_GET_STATUS)
    
    def parse_frame(self, data):
        """解析接收到的帧"""
        try:
            if len(data) < 9:
                return None
            
            # 检查帧头
            header = struct.unpack('>H', data[0:2])[0]
            if header != FRAME_HEADER:
                return None
            
            device_id = data[2]
            cmd_type = data[3]
            data_length = data[4]
            
            if len(data) < (9 + data_length):
                return None
            
            payload = data[5:5+data_length]
            
            # 检查CRC
            crc_received = struct.unpack('>H', data[5+data_length:7+data_length])[0]
            crc_calculated = self.calc_crc16(data[2:5+data_length])
            
            if crc_received != crc_calculated:
                print(f"CRC校验失败")
                return None
            
            # 检查帧尾
            tail = struct.unpack('>H', data[7+data_length:9+data_length])[0]
            if tail != FRAME_TAIL:
                return None
            
            return {
                'device_id': device_id,
                'cmd_type': cmd_type,
                'data': payload
            }
            
        except Exception as e:
            print(f"帧解析错误: {e}")
            return None
    
    def parse_lidar_data(self, data):
        """解析雷达扫描数据"""
        try:
            if len(data) < 6:
                return []
            
            # 解析头部
            point_count = struct.unpack('<H', data[0:2])[0]
            timestamp = struct.unpack('<I', data[2:6])[0]
            
            points = []
            offset = 6
            
            for i in range(point_count):
                if offset + 13 <= len(data):  # 每个点13字节 (4+4+1+4)
                    angle = struct.unpack('<f', data[offset:offset+4])[0]
                    distance = struct.unpack('<f', data[offset+4:offset+8])[0]
                    quality = data[offset+8]
                    start_flag = struct.unpack('<I', data[offset+9:offset+13])[0]
                    
                    points.append({
                        'angle': angle,
                        'distance': distance,
                        'quality': quality,
                        'start_flag': start_flag
                    })
                    offset += 13
            
            return points
            
        except Exception as e:
            print(f"雷达数据解析错误: {e}")
            return []
    
    def receive_data(self):
        """接收数据线程"""
        buffer = bytearray()
        
        while self.is_receiving:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # 查找完整帧
                    while len(buffer) >= 9:
                        # 查找帧头
                        header_pos = -1
                        for i in range(len(buffer) - 1):
                            if (buffer[i] << 8 | buffer[i+1]) == FRAME_HEADER:
                                header_pos = i
                                break
                        
                        if header_pos == -1:
                            buffer.clear()
                            break
                        
                        # 移除帧头前的数据
                        if header_pos > 0:
                            buffer = buffer[header_pos:]
                        
                        # 检查是否有完整帧
                        if len(buffer) >= 5:
                            data_length = buffer[4]
                            frame_length = 9 + data_length
                            
                            if len(buffer) >= frame_length:
                                frame_data = buffer[:frame_length]
                                buffer = buffer[frame_length:]
                                
                                # 解析帧
                                frame = self.parse_frame(frame_data)
                                if frame:
                                    self.handle_received_frame(frame)
                            else:
                                break
                        else:
                            break
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"接收数据错误: {e}")
                time.sleep(0.1)
    
    def handle_received_frame(self, frame):
        """处理接收到的帧"""
        cmd_type = frame['cmd_type']
        data = frame['data']
        
        if cmd_type == CMD_LIDAR_DATA:
            # 雷达扫描数据
            points = self.parse_lidar_data(data)
            with self.lock:
                self.scan_data = points
            print(f"收到雷达数据: {len(points)} 个点")
            
        elif cmd_type == CMD_SYSTEM_INFO:
            # 系统信息/雷达状态
            if len(data) >= 8:
                is_scanning, motor_running, health_status, scan_count = struct.unpack('<BBBL', data[:7])
                print(f"雷达状态: 扫描={is_scanning}, 电机={motor_running}, 健康={health_status}, 计数={scan_count}")
                
        elif cmd_type == CMD_ACK:
            # 命令确认
            if len(data) >= 1:
                ack_cmd = data[0]
                print(f"命令确认: 0x{ack_cmd:02X}")
    
    def start_receiving(self):
        """开始接收数据"""
        if self.is_receiving:
            return
        
        self.is_receiving = True
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
    
    def stop_receiving(self):
        """停止接收数据"""
        self.is_receiving = False
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=1)
    
    def get_scan_data(self):
        """获取最新扫描数据"""
        with self.lock:
            return self.scan_data.copy()
    
    def close(self):
        """关闭连接"""
        self.stop_receiving()
        if self.ser:
            self.ser.close()
            print("串口已关闭")

class LidarVisualizer:
    def __init__(self, controller):
        self.controller = controller
        self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        self.ax.set_ylim(0, 12)  # 最大12米
        self.ax.set_title("思岚C1雷达扫描数据", pad=20)
        
        self.points = self.ax.scatter([], [], c=[], s=[], alpha=0.6, cmap='viridis')
        
    def update_plot(self, frame):
        """更新图表"""
        scan_data = self.controller.get_scan_data()
        
        if scan_data:
            angles = []
            distances = []
            qualities = []
            sizes = []
            
            for point in scan_data:
                if point['distance'] > 0.1 and point['distance'] < 12:
                    angles.append(np.radians(point['angle']))
                    distances.append(point['distance'])
                    qualities.append(point['quality'])
                    sizes.append(max(10, point['quality']))
            
            if angles:
                self.points.set_offsets(np.column_stack((angles, distances)))
                self.points.set_array(np.array(qualities))
                self.points.set_sizes(sizes)
        
        return [self.points]
    
    def show(self):
        """显示图表"""
        ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)
        plt.show()

def main():
    """主函数"""
    print("=== 思岚C1雷达测试程序 ===")
    
    # 创建雷达控制器
    controller = LidarController('COM3', 9600)  # 根据实际端口修改
    
    if not controller.ser:
        print("无法连接到设备")
        return
    
    try:
        # 开始接收数据
        controller.start_receiving()
        
        # 控制序列
        print("1. 复位雷达...")
        controller.reset_lidar()
        time.sleep(2)
        
        print("2. 启动电机...")
        controller.start_motor()
        time.sleep(3)
        
        print("3. 获取状态...")
        controller.get_status()
        time.sleep(1)
        
        print("4. 开始扫描...")
        controller.start_scan()
        time.sleep(1)
        
        print("5. 启动可视化界面...")
        visualizer = LidarVisualizer(controller)
        
        # 显示实时数据
        print("显示实时雷达数据，关闭图表窗口结束程序...")
        visualizer.show()
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        print("停止扫描...")
        controller.stop_scan()
        time.sleep(1)
        
        print("停止电机...")
        controller.stop_motor()
        time.sleep(1)
        
        controller.close()

if __name__ == "__main__":
    main()
