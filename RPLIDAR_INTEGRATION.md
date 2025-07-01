# 思岚C1雷达集成使用文档

## 📋 概述

本文档详细说明了如何在STM32雷达车项目中集成和使用思岚C1激光雷达，包括数据接收、解析、控制和上位机通信功能。

## 🔧 硬件连接

### 雷达连接
```
思岚C1雷达          STM32F446xx
─────────────       ─────────────
VCC (5V)      ──►   5V电源
GND           ──►   GND
TXD           ──►   PA10 (USART1_RX)
RXD           ──►   PA9  (USART1_TX)
M_SCTP        ──►   PC0  (电机控制)
M_EN          ──►   3.3V (电机使能)
```

### 电源要求
- **工作电压**: 5V ±10%
- **工作电流**: 启动时≤1.5A，稳定时≤500mA
- **电机控制**: PC0输出高电平启动，低电平停止

## 📡 通信协议

### 雷达原生协议
思岚C1使用专有的二进制协议：

#### 命令格式
```
┌────────┬────────┬────────┬─────────┬──────────┐
│ 同步字  │ 命令标志│ 数据长度│  数据   │  校验和  │
├────────┼────────┼────────┼─────────┼──────────┤
│  0xA5  │   1字节 │  1字节  │ N字节   │  1字节   │
└────────┴────────┴────────┴─────────┴──────────┘
```

#### 应答格式
```
┌───────┬───────┬─────────────┬──────┬─────────┐
│同步字1 │同步字2 │ 大小&子类型  │ 类型  │  数据   │
├───────┼───────┼─────────────┼──────┼─────────┤
│ 0xA5  │ 0x5A  │   4字节     │1字节 │ N字节   │
└───────┴───────┴─────────────┴──────┴─────────┘
```

### 上位机通信协议
通过我们的标准通信协议与上位机交互：

#### 雷达控制命令
| 命令码 | 名称                | 说明           |
|-------|---------------------|----------------|
| 0x10  | CMD_LIDAR_START_SCAN| 开始扫描       |
| 0x11  | CMD_LIDAR_STOP_SCAN | 停止扫描       |
| 0x12  | CMD_LIDAR_START_MOTOR| 启动电机      |
| 0x13  | CMD_LIDAR_STOP_MOTOR| 停止电机       |
| 0x14  | CMD_LIDAR_RESET     | 雷达复位       |
| 0x15  | CMD_LIDAR_GET_STATUS| 获取雷达状态   |

## 🚀 软件架构

### 任务分工
```
┌─────────────────────────────────────────────────────────┐
│                    FreeRTOS 任务层                       │
├─────────────────┬─────────────────┬─────────────────────┤
│   lidarTask     │   commTask      │   其他任务           │
│  (雷达数据处理)  │  (通信管理)      │                     │
└─────────────────┴─────────────────┴─────────────────────┘
         │                    ▲
         ▼                    │
┌─────────────────────────────────────────────────────────┐
│                 雷达驱动层                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │ 协议解析    │  │ 数据转换    │  │ 命令控制    │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────┘
         │                    ▲
         ▼                    │
┌─────────────────────────────────────────────────────────┐
│                   UART1 驱动                            │
│        115200 bps, 8N1, 中断接收                        │
└─────────────────────────────────────────────────────────┘
```

### 核心模块

#### 1. rplidar.h/c - 雷达驱动模块
**功能**:
- 雷达协议解析
- 扫描数据处理
- 电机控制
- 状态管理

**关键函数**:
```c
void RPLidar_Init(void);                    // 初始化雷达
void RPLidar_StartScan(void);               // 开始扫描
void RPLidar_StopScan(void);                // 停止扫描
void RPLidar_ProcessRxByte(uint8_t byte);   // 处理接收字节
uint8_t RPLidar_ParseResponse(void);        // 解析应答数据
```

#### 2. 数据结构

##### 扫描点数据
```c
typedef struct {
    float angle;        // 角度 (度, 0-360)
    float distance;     // 距离 (米, 0.1-12.0)
    uint8_t quality;    // 信号质量 (0-63)
    uint8_t start_flag; // 起始标志
} rplidar_scan_point_t;
```

##### 雷达状态
```c
typedef struct {
    uint8_t is_scanning;    // 是否正在扫描
    uint8_t motor_running;  // 电机运行状态
    uint8_t health_status;  // 健康状态
    uint32_t scan_count;    // 扫描计数
} rplidar_status_t;
```

## 💻 使用示例

### 1. 初始化和启动
```c
// 在lidarTask中
void lidarTask(void *argument) {
    // 初始化雷达
    RPLidar_Init();
    
    // 等待初始化完成
    osDelay(2000);
    
    // 主循环
    for(;;) {
        // 解析数据
        if (RPLidar_ParseResponse()) {
            // 数据已自动发送到上位机
        }
        
        osDelay(10);
    }
}
```

### 2. 上位机控制命令
```python
# Python示例
import serial
import struct

class LidarController:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600)
    
    def start_scan(self):
        # 发送开始扫描命令
        frame = self.pack_frame(0x04, 0x10, b'')
        self.ser.write(frame)
    
    def stop_scan(self):
        # 发送停止扫描命令
        frame = self.pack_frame(0x04, 0x11, b'')
        self.ser.write(frame)
```

### 3. 数据接收和可视化
```python
def parse_lidar_data(self, data):
    """解析雷达扫描数据"""
    point_count = struct.unpack('<H', data[0:2])[0]
    timestamp = struct.unpack('<I', data[2:6])[0]
    
    points = []
    offset = 6
    
    for i in range(point_count):
        if offset + 13 <= len(data):
            angle = struct.unpack('<f', data[offset:offset+4])[0]
            distance = struct.unpack('<f', data[offset+4:offset+8])[0]
            quality = data[offset+8]
            
            points.append({
                'angle': angle,
                'distance': distance,
                'quality': quality
            })
            offset += 13
    
    return points
```

## 🔍 测试和调试

### 1. 运行测试脚本
```bash
# 安装依赖
pip install pyserial matplotlib numpy

# 运行雷达测试
python test_lidar.py
```

### 2. 测试序列
1. **复位雷达** - 确保雷达处于已知状态
2. **启动电机** - 等待电机稳定运行
3. **开始扫描** - 启动数据采集
4. **实时显示** - 极坐标图显示扫描结果

### 3. 状态监控
```c
// 在调试时添加状态输出
void RPLidar_SendStatusToPC(void) {
    printf("雷达状态: 扫描=%d, 电机=%d, 健康=%d\n", 
           rplidar_status.is_scanning,
           rplidar_status.motor_running,
           rplidar_status.health_status);
}
```

## ⚠️ 注意事项

### 1. 硬件注意事项
- **电源要求**: 必须提供稳定的5V电源
- **启动电流**: 启动瞬间电流较大，需要足够的电源容量
- **散热**: 长时间运行需要注意散热
- **防尘**: 避免灰尘进入激光发射窗口

### 2. 软件注意事项
- **初始化时间**: 雷达启动需要1-2秒稳定时间
- **数据量**: 扫描数据量较大，注意内存和带宽管理
- **同步**: 确保电机启动后再开始扫描
- **错误处理**: 实现超时和错误恢复机制

### 3. 通信注意事项
- **波特率**: 雷达使用115200bps，蓝牙使用9600bps
- **数据包大小**: 单次扫描可产生大量数据点
- **实时性**: 优化数据传输以保证实时性

## 🚨 故障排除

### 常见问题

#### 1. 雷达无响应
**症状**: 发送命令后无任何回应
**排查步骤**:
```c
// 检查串口配置
if (huart1.Init.BaudRate != 115200) {
    Error_Handler();
}

// 检查GPIO配置
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
osDelay(100);

// 发送复位命令
RPLidar_Reset();
```

#### 2. 电机不启动
**症状**: 发送启动命令但电机不转
**解决方法**:
- 检查电源电压是否为5V
- 确认PC0 GPIO输出高电平
- 检查M_EN引脚连接

#### 3. 数据解析错误
**症状**: 接收到数据但解析失败
**调试代码**:
```c
void debug_rx_data(uint8_t *data, uint16_t len) {
    printf("RX: ");
    for(int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}
```

#### 4. 扫描数据异常
**症状**: 扫描点数据不合理
**检查要点**:
- 角度范围: 0-360度
- 距离范围: 0.1-12米
- 质量值: 0-63

### 性能优化

#### 1. 减少数据传输量
```c
// 只发送质量较好的点
if (points[i].quality > 10 && points[i].distance > 0.2f) {
    // 添加到发送缓冲区
}
```

#### 2. 优化接收缓冲区
```c
// 增大缓冲区以应对突发数据
#define RPLIDAR_RX_BUFFER_SIZE 1024
```

#### 3. 数据压缩
```c
// 对角度和距离进行适当量化
uint16_t angle_q = (uint16_t)(angle * 10);  // 0.1度精度
uint16_t dist_q = (uint16_t)(distance * 1000); // 1mm精度
```

## 📈 性能指标

### 雷达规格
- **测距范围**: 0.1 - 12米
- **角度分辨率**: 1度
- **测距精度**: ±1%
- **扫描频率**: 2-10Hz
- **角度范围**: 360度

### 系统性能
- **数据延迟**: < 100ms
- **CPU占用**: < 10%
- **内存使用**: 约2KB
- **通信带宽**: 约1KB/s (取决于点数)

## 🔄 扩展功能

### 1. 地图构建
```c
// 添加SLAM算法支持
typedef struct {
    float x, y;     // 世界坐标
    float confidence; // 置信度
} map_point_t;
```

### 2. 障碍物检测
```c
// 检测前方障碍物
uint8_t detect_obstacle(rplidar_scan_point_t *points, uint16_t count) {
    for(uint16_t i = 0; i < count; i++) {
        if (points[i].angle > 350 || points[i].angle < 10) {
            if (points[i].distance < 0.5f) { // 50cm内有障碍
                return 1;
            }
        }
    }
    return 0;
}
```

### 3. 数据录制
```c
// 将扫描数据保存到SD卡
void save_scan_data(rplidar_scan_point_t *points, uint16_t count);
```

## 📝 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v1.0 | 2025-07-01 | 初始版本，基础雷达功能 |
| v1.1 | - | 添加数据可视化和优化 |
| v1.2 | - | 增加错误处理和状态监控 |

---

*本文档最后更新: 2025年7月1日*
