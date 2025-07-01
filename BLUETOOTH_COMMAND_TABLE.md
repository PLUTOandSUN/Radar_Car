# 蓝牙通信指令表

## 📋 通信协议格式

```
[帧头][设备ID][命令类型][数据长度][数据][CRC16][帧尾]
2字节   1字节    1字节     1字节    N字节  2字节   2字节
0x55AA  0x01-04  0x00-FF   0-64     Data   CRC    0x0D0A
```

## 🆔 设备ID定义

| 设备ID | 设备名称 | 说明 |
|--------|----------|------|
| 0x01   | STM32    | 本机微控制器 |
| 0x02   | 蓝牙模块 | HC-05/HC-06 |
| 0x03   | 雷达     | 激光雷达模块 |
| 0x04   | 上位机   | PC/手机应用 |

## 📝 命令类型详细表

### 基础通信命令 (0x00-0x0F)

| 命令码 | 命令名称 | 方向 | 数据长度 | 数据格式 | 说明 |
|--------|----------|------|----------|----------|------|
| 0x00   | CMD_HEARTBEAT | 双向 | 4字节 | uint32_t timestamp | 心跳包，保持连接活跃 |
| 0x0A   | CMD_ACK | 上行 | 1字节 | uint8_t cmd_code | 命令确认应答 |
| 0x0B   | CMD_NACK | 上行 | 2字节 | uint8_t cmd_code + uint8_t error | 命令拒绝应答 |

### 电机控制命令 (0x01-0x02)

| 命令码 | 命令名称 | 方向 | 数据长度 | 数据格式 | 说明 |
|--------|----------|------|----------|----------|------|
| 0x01   | CMD_MOTOR_CTRL | 下行 | 9字节 | MotorCtrlData_t | 电机速度控制 |
| 0x02   | CMD_MOTOR_STATUS | 上行 | 17字节 | MotorStatusData_t | 电机状态反馈 |

#### MotorCtrlData_t 结构 (9字节)
```c
float left_speed;    // 左轮目标速度 (RPM)  [4字节]
float right_speed;   // 右轮目标速度 (RPM)  [4字节]  
uint8_t direction;   // 方向控制            [1字节]
```

#### 方向控制值定义
- `0x00` - 停止
- `0x01` - 前进
- `0x02` - 后退

#### MotorStatusData_t 结构 (17字节)
```c
float left_speed;     // 左轮当前速度 (RPM) [4字节]
float right_speed;    // 右轮当前速度 (RPM) [4字节]
float left_current;   // 左轮电流 (A)       [4字节]
float right_current;  // 右轮电流 (A)       [4字节]
uint8_t status;       // 状态标志位         [1字节]
```

#### 电机状态标志位
- `0x01` - MOTOR_STATUS_NORMAL (正常运行)
- `0x02` - MOTOR_STATUS_ERROR (错误状态)
- `0x04` - MOTOR_STATUS_OVERLOAD (过载保护)
- `0x08` - MOTOR_STATUS_STALL (堵转保护)

### 传感器数据命令 (0x03-0x05)

| 命令码 | 命令名称 | 方向 | 数据长度 | 数据格式 | 说明 |
|--------|----------|------|----------|----------|------|
| 0x03   | CMD_IMU_DATA | 上行 | 40字节 | IMUData_t | IMU传感器数据 |
| 0x04   | CMD_LIDAR_DATA | 上行 | 变长 | LidarDataPacket_t | 雷达扫描数据 |
| 0x05   | CMD_ODOM_DATA | 上行 | 24字节 | OdomData_t | 里程计数据 |

#### IMUData_t 结构 (40字节)
```c
float accel_x, accel_y, accel_z;    // 加速度 (m/s²)     [12字节]
float gyro_x, gyro_y, gyro_z;       // 角速度 (rad/s)    [12字节]
float mag_x, mag_y, mag_z;          // 磁力计 (μT)       [12字节]
float temperature;                  // 温度 (°C)         [4字节]
```

#### OdomData_t 结构 (24字节)
```c
float x, y;           // 位置坐标 (m)      [8字节]
float theta;          // 朝向角 (rad)      [4字节]
float linear_vel;     // 线速度 (m/s)      [4字节]
float angular_vel;    // 角速度 (rad/s)    [4字节]
uint32_t timestamp;   // 时间戳 (ms)       [4字节]
```

### 系统管理命令 (0x06-0x09)

| 命令码 | 命令名称 | 方向 | 数据长度 | 数据格式 | 说明 |
|--------|----------|------|----------|----------|------|
| 0x06   | CMD_SET_PARAM | 下行 | 变长 | param_id + value | 参数设置 |
| 0x07   | CMD_GET_PARAM | 下行 | 1字节 | uint8_t param_id | 参数获取请求 |
| 0x08   | CMD_SYSTEM_INFO | 上行 | 变长 | SystemInfo_t | 系统信息 |
| 0x09   | CMD_ERROR_REPORT | 上行 | 变长 | ErrorReport_t | 错误报告 |

### 雷达专用命令 (0x10-0x15)

| 命令码 | 命令名称 | 方向 | 数据长度 | 数据格式 | 说明 |
|--------|----------|------|----------|----------|------|
| 0x10   | CMD_LIDAR_START_SCAN | 下行 | 0字节 | 无 | 开始扫描 |
| 0x11   | CMD_LIDAR_STOP_SCAN | 下行 | 0字节 | 无 | 停止扫描 |
| 0x12   | CMD_LIDAR_START_MOTOR | 下行 | 0字节 | 无 | 启动电机 |
| 0x13   | CMD_LIDAR_STOP_MOTOR | 下行 | 0字节 | 无 | 停止电机 |
| 0x14   | CMD_LIDAR_RESET | 下行 | 0字节 | 无 | 雷达复位 |
| 0x15   | CMD_LIDAR_GET_STATUS | 下行 | 0字节 | 无 | 获取雷达状态 |

## 🔧 CRC16校验

- **算法**: CCITT标准 (多项式: 0x1021)
- **初始值**: 0xFFFF
- **校验范围**: 设备ID + 命令类型 + 数据长度 + 数据
- **字节序**: 大端序 (高字节在前)

## 📝 使用示例

### 发送电机控制命令
```
帧头: 55 AA
设备ID: 04 (PC)
命令: 01 (电机控制)
长度: 09
数据: 00 00 48 42 (50.0f左轮) 00 00 48 42 (50.0f右轮) 01 (前进)
CRC: XX XX
帧尾: 0D 0A
```

### 接收电机状态
```
帧头: 55 AA  
设备ID: 01 (STM32)
命令: 02 (电机状态)
长度: 11
数据: 当前速度+电流+状态
CRC: XX XX
帧尾: 0D 0A
```

### 心跳包
```
帧头: 55 AA
设备ID: 01 (STM32)  
命令: 00 (心跳)
长度: 04
数据: XX XX XX XX (时间戳)
CRC: XX XX
帧尾: 0D 0A
```

## ⚠️ 注意事项

1. **浮点数格式**: 使用IEEE 754标准32位浮点数
2. **字节序**: 小端序存储 (除CRC和帧头/帧尾)
3. **超时设置**: 接收超时 100ms，心跳间隔 1000ms
4. **缓冲区大小**: 最大数据长度 64字节
5. **并发保护**: 发送时使用UART互斥锁
6. **错误处理**: CRC失败时发送NACK应答

## 📊 性能指标

- **最大帧长**: 73字节
- **最小帧长**: 9字节
- **通信延迟**: < 50ms (蓝牙)
- **处理周期**: 10ms
- **可靠性**: 99.9984% (CRC16检错)

---
*文档版本: v1.0 | 最后更新: 2025-07-01*
