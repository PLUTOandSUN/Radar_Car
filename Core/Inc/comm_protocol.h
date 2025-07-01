/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : comm_protocol.h
  * @brief          : 通信协议定义与蓝牙通信管理头文件
  ******************************************************************************
  * @attention
  *
  * 通信协议格式:
  * [帧头][设备ID][命令类型][数据长度][数据][CRC16][帧尾]
  * 2字节   1字节    1字节     1字节    N字节  2字节   2字节
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __COMM_PROTOCOL_H
#define __COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>

/* 通信协议定义 */
#define FRAME_HEADER        0x55AA      // 帧头
#define FRAME_TAIL          0x0D0A      // 帧尾 (CR LF)
#define MAX_DATA_LENGTH     64          // 最大数据长度
#define FRAME_MIN_LENGTH    9           // 最小帧长度(不含数据)

/* 设备ID定义 */
#define DEVICE_STM32        0x01        // STM32设备
#define DEVICE_BLUETOOTH    0x02        // 蓝牙设备
#define DEVICE_LIDAR        0x03        // 雷达设备
#define DEVICE_PC           0x04        // 上位机

/* 命令类型定义 */
typedef enum {
    CMD_HEARTBEAT       = 0x00,         // 心跳包
    CMD_MOTOR_CTRL      = 0x01,         // 电机控制
    CMD_MOTOR_STATUS    = 0x02,         // 电机状态
    CMD_IMU_DATA        = 0x03,         // IMU数据
    CMD_LIDAR_DATA      = 0x04,         // 雷达数据
    CMD_ODOM_DATA       = 0x05,         // 里程计数据
    CMD_SET_PARAM       = 0x06,         // 参数设置
    CMD_GET_PARAM       = 0x07,         // 参数获取
    CMD_SYSTEM_INFO     = 0x08,         // 系统信息
    CMD_ERROR_REPORT    = 0x09,         // 错误报告
    CMD_ACK             = 0x0A,         // 应答
    CMD_NACK            = 0x0B,         // 否定应答
    CMD_LIDAR_START_SCAN    = 0x10,         // 开始扫描
    CMD_LIDAR_STOP_SCAN     = 0x11,         // 停止扫描
    CMD_LIDAR_START_MOTOR   = 0x12,         // 启动电机
    CMD_LIDAR_STOP_MOTOR    = 0x13,         // 停止电机
    CMD_LIDAR_RESET         = 0x14,         // 雷达复位
    CMD_LIDAR_GET_STATUS    = 0x15          // 获取雷达状态
} CommandType_t;

/* 通信帧结构体 */
typedef struct {
    uint16_t header;                    // 帧头
    uint8_t device_id;                  // 设备ID
    uint8_t cmd_type;                   // 命令类型
    uint8_t data_length;                // 数据长度
    uint8_t data[MAX_DATA_LENGTH];      // 数据
    uint16_t crc;                       // CRC校验
    uint16_t tail;                      // 帧尾
} CommFrame_t;

/* 电机控制数据结构 */
typedef struct {
    float left_speed;                   // 左轮速度(RPM)
    float right_speed;                  // 右轮速度(RPM)
    uint8_t direction;                  // 方向控制
} MotorCtrlData_t;

/* 电机状态数据结构 */
typedef struct {
    float left_speed;                   // 左轮当前速度
    float right_speed;                  // 右轮当前速度
    float left_current;                 // 左轮电流
    float right_current;                // 右轮电流
    uint8_t status;                     // 状态标志
} MotorStatusData_t;

/* IMU数据结构 */
typedef struct {
    float accel_x, accel_y, accel_z;    // 加速度
    float gyro_x, gyro_y, gyro_z;       // 角速度
    float mag_x, mag_y, mag_z;          // 磁力计
    float temperature;                  // 温度
} IMUData_t;

/* 里程计数据结构 */
typedef struct {
    float x, y, theta;                  // 位置和角度
    float linear_vel, angular_vel;      // 线速度和角速度
    uint32_t timestamp;                 // 时间戳
} OdomData_t;

/* 雷达数据结构 */
typedef struct {
    uint16_t angle;                     // 角度(0.01度)
    uint16_t distance;                  // 距离(mm)
    uint8_t quality;                    // 质量
} LidarPoint_t;

/* 雷达扫描点数据结构 */
typedef struct {
    float angle;                    // 角度 (度)
    float distance;                 // 距离 (米)
    uint8_t quality;                // 信号质量 (0-63)
    uint8_t start_flag;             // 起始标志
} LidarScanPoint_t;

/* 雷达数据包结构 */
typedef struct {
    uint16_t point_count;           // 点数量
    uint32_t timestamp;             // 时间戳
    LidarScanPoint_t points[360];   // 扫描点数据
} LidarDataPacket_t;

/* 雷达状态数据结构 */
typedef struct {
    uint8_t is_scanning;            // 是否正在扫描
    uint8_t motor_running;          // 电机运行状态
    uint8_t health_status;          // 健康状态
    uint32_t scan_count;            // 扫描计数
    uint8_t model;                  // 雷达型号
    uint16_t firmware_version;      // 固件版本
} LidarStatusData_t;

/* 通信状态枚举 */
typedef enum {
    COMM_IDLE,
    COMM_RECEIVING,
    COMM_FRAME_READY,
    COMM_ERROR
} CommState_t;

/* 接收缓冲区结构 */
typedef struct {
    uint8_t buffer[256];                // 接收缓冲区
    uint16_t write_index;               // 写指针
    uint16_t read_index;                // 读指针
    CommState_t state;                  // 接收状态
    uint32_t last_byte_time;            // 最后接收字节时间
} RxBuffer_t;

/* 外部变量声明 */
extern RxBuffer_t bt_rx_buffer;         // 蓝牙接收缓冲区
extern RxBuffer_t lidar_rx_buffer;      // 雷达接收缓冲区
extern osMessageQueueId_t btCommQueueHandle;
extern osMessageQueueId_t lidarCommQueueHandle;

/* 函数声明 */
uint16_t CRC16_Calculate(uint8_t *data, uint16_t length);
uint8_t CommFrame_Pack(CommFrame_t *frame, uint8_t *buffer);
uint8_t CommFrame_Unpack(uint8_t *buffer, uint16_t length, CommFrame_t *frame);
uint8_t CommFrame_Validate(CommFrame_t *frame);

void BT_SendFrame(uint8_t device_id, uint8_t cmd_type, uint8_t *data, uint8_t length);
void Lidar_SendFrame(uint8_t device_id, uint8_t cmd_type, uint8_t *data, uint8_t length);

void BT_ProcessRxData(uint8_t data);
void Lidar_ProcessRxData(uint8_t data);
uint8_t BT_ParseFrame(CommFrame_t *frame);
uint8_t Lidar_ParseFrame(CommFrame_t *frame);

void Comm_HandleCommand(CommFrame_t *frame, uint8_t source);
void Comm_SendMotorStatus(void);
void Comm_SendIMUData(void);
void Comm_SendOdomData(void);
void Comm_SendHeartbeat(void);

void Comm_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H */
