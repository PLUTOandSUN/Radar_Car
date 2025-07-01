#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝牙通信测试脚本
用于测试STM32雷达车的蓝牙通信协议
"""

import serial
import struct
import time
import crc16

# 通信协议常量
FRAME_HEADER = 0x55AA
FRAME_TAIL = 0x0D0A
DEVICE_STM32 = 0x01
DEVICE_PC = 0x04

# 命令类型
CMD_HEARTBEAT = 0x00
CMD_MOTOR_CTRL = 0x01
CMD_MOTOR_STATUS = 0x02
CMD_IMU_DATA = 0x03
CMD_GET_PARAM = 0x07

class BluetoothComm:
    def __init__(self, port='COM3', baudrate=9600):
        """初始化蓝牙串口"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"蓝牙串口已连接: {port} @ {baudrate}")
        except Exception as e:
            print(f"串口连接失败: {e}")
            self.ser = None
    
    def calc_crc16(self, data):
        """计算CRC16校验值 (CCITT标准)"""
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
        
        # 计算CRC (从设备ID到数据结束)
        crc_data = frame[2:]  # 跳过帧头
        crc = self.calc_crc16(crc_data)
        frame.extend(struct.pack('>H', crc))
        
        # 帧尾
        frame.extend(struct.pack('>H', FRAME_TAIL))
        
        return frame
    
    def unpack_frame(self, frame_data):
        """解包通信帧"""
        if len(frame_data) < 9:
            return None
        
        try:
            # 解析帧头
            header = struct.unpack('>H', frame_data[0:2])[0]
            if header != FRAME_HEADER:
                return None
            
            # 解析字段
            device_id = frame_data[2]
            cmd_type = frame_data[3]
            data_length = frame_data[4]
            
            if len(frame_data) < (9 + data_length):
                return None
            
            data = frame_data[5:5+data_length]
            
            # 检查CRC
            crc_received = struct.unpack('>H', frame_data[5+data_length:7+data_length])[0]
            crc_calculated = self.calc_crc16(frame_data[2:5+data_length])
            
            if crc_received != crc_calculated:
                print(f"CRC校验失败: 接收={crc_received:04X}, 计算={crc_calculated:04X}")
                return None
            
            # 检查帧尾
            tail = struct.unpack('>H', frame_data[7+data_length:9+data_length])[0]
            if tail != FRAME_TAIL:
                return None
            
            return {
                'device_id': device_id,
                'cmd_type': cmd_type,
                'data': data
            }
            
        except Exception as e:
            print(f"帧解析错误: {e}")
            return None
    
    def send_motor_ctrl(self, left_speed, right_speed, direction=0):
        """发送电机控制命令"""
        # 打包电机控制数据
        data = struct.pack('<ffB', left_speed, right_speed, direction)
        frame = self.pack_frame(DEVICE_PC, CMD_MOTOR_CTRL, data)
        
        if self.ser:
            self.ser.write(frame)
            print(f"发送电机控制: 左轮={left_speed}, 右轮={right_speed}")
    
    def send_get_param(self, param_type):
        """发送参数获取命令"""
        data = struct.pack('B', param_type)
        frame = self.pack_frame(DEVICE_PC, CMD_GET_PARAM, data)
        
        if self.ser:
            self.ser.write(frame)
            print(f"请求参数: {param_type}")
    
    def send_heartbeat(self):
        """发送心跳包"""
        timestamp = int(time.time() * 1000)
        data = struct.pack('<I', timestamp)
        frame = self.pack_frame(DEVICE_PC, CMD_HEARTBEAT, data)
        
        if self.ser:
            self.ser.write(frame)
            print("发送心跳包")
    
    def receive_data(self):
        """接收并解析数据"""
        if not self.ser or self.ser.in_waiting == 0:
            return None
        
        # 简单的帧同步 - 查找帧头
        buffer = bytearray()
        start_time = time.time()
        
        while time.time() - start_time < 1.0:  # 1秒超时
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte:
                    buffer.extend(byte)
                    
                    # 查找帧头
                    if len(buffer) >= 2:
                        header = struct.unpack('>H', buffer[-2:])[0]
                        if header == FRAME_HEADER:
                            # 找到帧头，开始接收完整帧
                            frame_start = len(buffer) - 2
                            
                            # 等待接收至少9字节的最小帧
                            while len(buffer) < frame_start + 9:
                                if self.ser.in_waiting > 0:
                                    buffer.extend(self.ser.read(1))
                                else:
                                    time.sleep(0.001)
                            
                            # 获取数据长度
                            data_length = buffer[frame_start + 4]
                            expected_length = 9 + data_length
                            
                            # 接收完整帧
                            while len(buffer) < frame_start + expected_length:
                                if self.ser.in_waiting > 0:
                                    buffer.extend(self.ser.read(1))
                                else:
                                    time.sleep(0.001)
                            
                            # 解析帧
                            frame_data = buffer[frame_start:frame_start + expected_length]
                            return self.unpack_frame(frame_data)
            else:
                time.sleep(0.001)
        
        return None
    
    def close(self):
        """关闭串口"""
        if self.ser:
            self.ser.close()
            print("蓝牙串口已关闭")

def main():
    """主测试函数"""
    print("=== STM32雷达车蓝牙通信测试 ===")
    
    # 创建蓝牙通信对象 (请根据实际情况修改COM端口)
    bt = BluetoothComm('COM3', 9600)
    
    if not bt.ser:
        print("无法连接到蓝牙模块，请检查串口设置")
        return
    
    try:
        # 测试序列
        test_count = 0
        
        while test_count < 10:  # 运行10轮测试
            print(f"\n--- 测试轮次 {test_count + 1} ---")
            
            # 1. 发送心跳包
            bt.send_heartbeat()
            time.sleep(0.1)
            
            # 2. 请求电机状态
            bt.send_get_param(0x01)  # 0x01表示电机状态
            time.sleep(0.1)
            
            # 3. 发送电机控制命令
            left_speed = 30.0 + test_count * 5
            right_speed = 30.0 + test_count * 5
            bt.send_motor_ctrl(left_speed, right_speed)
            time.sleep(0.1)
            
            # 4. 接收响应数据
            response = bt.receive_data()
            if response:
                cmd_type = response['cmd_type']
                data = response['data']
                
                if cmd_type == CMD_MOTOR_STATUS and len(data) >= 17:
                    # 解析电机状态
                    left_speed, right_speed, left_current, right_current, status = struct.unpack('<ffffB', data)
                    print(f"电机状态: 左轮={left_speed:.1f}RPM, 右轮={right_speed:.1f}RPM, 状态={status}")
                elif cmd_type == CMD_HEARTBEAT:
                    timestamp = struct.unpack('<I', data)[0] if len(data) >= 4 else 0
                    print(f"收到心跳包: {timestamp}")
                else:
                    print(f"收到命令 0x{cmd_type:02X}, 数据长度: {len(data)}")
            else:
                print("未收到响应数据")
            
            test_count += 1
            time.sleep(1)  # 1秒间隔
            
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        bt.close()

if __name__ == "__main__":
    main()
