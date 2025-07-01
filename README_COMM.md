# STM32雷达车蓝牙通信系统

## 系统概述

本项目实现了一个完整的STM32雷达车蓝牙通信系统，支持与上位机的双向通信，包含完整的通信协议、路由机制和CRC校验。

## 通信协议规范

### 帧格式
```
[帧头][设备ID][命令类型][数据长度][数据][CRC16][帧尾]
2字节   1字节    1字节     1字节    N字节  2字节   2字节
```

### 协议详细说明
- **帧头**: 0x55AA (固定)
- **设备ID**: 设备标识符
  - 0x01: STM32设备
  - 0x02: 蓝牙设备  
  - 0x03: 雷达设备
  - 0x04: 上位机
- **命令类型**: 见命令列表
- **数据长度**: 数据字段的字节数 (0-64)
- **数据**: 具体的数据内容
- **CRC16**: CCITT标准校验码
- **帧尾**: 0x0D0A (CR LF)

### 命令类型列表

| 命令码 | 名称 | 说明 | 数据格式 |
|--------|------|------|----------|
| 0x00 | CMD_HEARTBEAT | 心跳包 | 4字节时间戳 |
| 0x01 | CMD_MOTOR_CTRL | 电机控制 | float left_speed, float right_speed, uint8_t direction |
| 0x02 | CMD_MOTOR_STATUS | 电机状态 | float left_speed, float right_speed, float left_current, float right_current, uint8_t status |
| 0x03 | CMD_IMU_DATA | IMU数据 | 10个float值(加速度、角速度、磁力计、温度) |
| 0x05 | CMD_ODOM_DATA | 里程计数据 | float x, y, theta, linear_vel, angular_vel, uint32_t timestamp |
| 0x07 | CMD_GET_PARAM | 参数获取 | uint8_t param_type |
| 0x0A | CMD_ACK | 应答 | 可变 |
| 0x0B | CMD_NACK | 否定应答 | uint8_t error_cmd |

## 硬件配置

### 串口分配
- **USART1**: 雷达通信 (115200 bps)
- **USART3**: 蓝牙通信 (9600 bps) - 使用DMA
- **USART2**: 调试输出 (115200 bps)

### GPIO配置
- **PWM输出**: TIM2_CH1(PA0), TIM2_CH2(PA1) - 电机控制
- **编码器**: TIM3(PA6,PA7), TIM4(PB6,PB7) - 速度反馈

## 软件架构

### FreeRTOS任务
1. **commTask** (优先级: Low)
   - 处理蓝牙和雷达通信
   - 解析接收到的命令帧
   - 定期发送状态信息
   - 心跳包管理

2. **motorCtrlTask** (优先级: Low)
   - 电机PID控制
   - 接收通信队列的控制命令
   - PWM输出管理

3. **其他任务**
   - imuTask: IMU数据采集
   - lidarTask: 雷达数据处理
   - odomTask: 里程计计算

### 通信队列
- **cmdQueue**: 电机控制命令队列
- **btCommQueue**: 蓝牙通信帧队列
- **lidarCommQueue**: 雷达通信帧队列

## 使用说明

### 1. 编译和烧录
```bash
# 使用CMake构建
cd build
cmake ..
make
# 烧录到STM32
```

### 2. 蓝牙配置
1. 连接HC-05/HC-06蓝牙模块到USART3
2. 配置蓝牙模块波特率为9600
3. 与上位机配对连接

### 3. 测试通信
使用提供的Python测试脚本:
```bash
# 安装依赖
pip install pyserial

# 运行测试脚本
python test_bluetooth.py
```

### 4. 上位机通信示例

#### 发送电机控制命令
```python
import struct

# 构建电机控制数据 (左轮50RPM, 右轮50RPM, 前进)
data = struct.pack('<ffB', 50.0, 50.0, 1)
frame = pack_frame(DEVICE_PC, CMD_MOTOR_CTRL, data)
serial.write(frame)
```

#### 请求电机状态
```python
# 请求电机状态
data = struct.pack('B', 0x01)  # 0x01表示电机状态
frame = pack_frame(DEVICE_PC, CMD_GET_PARAM, data)
serial.write(frame)
```

## API参考

### 核心函数

#### `Comm_Init()`
初始化通信系统，启动UART中断接收。

#### `BT_SendFrame(device_id, cmd_type, data, length)`
通过蓝牙发送数据帧。

#### `Comm_HandleCommand(frame, source)`
处理接收到的命令，实现命令路由。

#### `CRC16_Calculate(data, length)`
计算CRC16校验码。

### 数据结构

#### `CommFrame_t`
通信帧结构体，包含所有帧字段。

#### `MotorCtrlData_t`
电机控制数据结构。

#### `MotorStatusData_t`
电机状态数据结构。

## 故障排除

### 常见问题

1. **通信无响应**
   - 检查串口连接和波特率设置
   - 确认蓝牙模块已正确配对
   - 检查帧格式和CRC校验

2. **电机不动作**
   - 确认PWM输出配置正确
   - 检查电机驱动电路
   - 验证PID参数设置

3. **数据接收错误**
   - 检查UART中断是否正常
   - 确认DMA配置正确
   - 查看接收缓冲区状态

### 调试建议

1. 使用串口调试工具监控通信数据
2. 添加LED指示器显示通信状态
3. 通过USART2输出调试信息
4. 使用逻辑分析仪检查信号

## 扩展功能

### 计划功能
1. 加密通信支持
2. 多设备路由
3. 固件升级功能
4. 配置参数存储

### 自定义命令
可以通过修改`CommandType_t`枚举和`Comm_HandleCommand`函数来添加新的通信命令。

## 版本历史

- v1.0: 基础通信协议实现
- v1.1: 添加CRC校验和路由机制
- v1.2: 完善错误处理和超时机制

## 许可证

本项目采用MIT许可证，详见LICENSE文件。
