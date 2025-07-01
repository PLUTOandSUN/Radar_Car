/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : rplidar.h
  * @brief          : 思岚C1激光雷达驱动头文件
  ******************************************************************************
  * @attention
  *
  * 思岚C1雷达通信协议实现
  * - 支持扫描数据接收和解析
  * - 支持电机控制命令
  * - 数据转发到上位机
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/* 雷达协议定义 */
#define RPLIDAR_CMD_SYNC_BYTE           0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD     0x80

#define RPLIDAR_ANS_SYNC_BYTE1          0xA5
#define RPLIDAR_ANS_SYNC_BYTE2          0x5A

/* 雷达命令定义 */
#define RPLIDAR_CMD_STOP                0x25
#define RPLIDAR_CMD_RESET               0x40
#define RPLIDAR_CMD_SCAN                0x20
#define RPLIDAR_CMD_EXPRESS_SCAN        0x82
#define RPLIDAR_CMD_FORCE_SCAN          0x21
#define RPLIDAR_CMD_GET_INFO            0x50
#define RPLIDAR_CMD_GET_HEALTH          0x52

/* 应答类型定义 */
#define RPLIDAR_ANS_TYPE_DEVINFO        0x4
#define RPLIDAR_ANS_TYPE_DEVHEALTH      0x6
#define RPLIDAR_ANS_TYPE_MEASUREMENT    0x81
#define RPLIDAR_ANS_TYPE_SAMPLE_RATE    0x15

/* 雷达状态定义 */
#define RPLIDAR_STATUS_OK               0x0
#define RPLIDAR_STATUS_WARNING          0x1
#define RPLIDAR_STATUS_ERROR            0x2

/* 数据缓冲区大小 */
#define RPLIDAR_RX_BUFFER_SIZE          512
#define RPLIDAR_MAX_SCAN_POINTS         360

/* 雷达命令结构体 */
typedef struct {
    uint8_t sync_byte;          // 同步字节 0xA5
    uint8_t cmd_flag;           // 命令标志
    uint8_t size;               // 数据大小
    uint8_t data[255];          // 数据内容
    uint8_t checksum;           // 校验和
} __attribute__((packed)) rplidar_cmd_packet_t;

/* 雷达应答头结构体 */
typedef struct {
    uint8_t sync_byte1;         // 同步字节1 0xA5
    uint8_t sync_byte2;         // 同步字节2 0x5A
    uint32_t size_q30_subtype;  // 大小和子类型
    uint8_t type;               // 类型
} __attribute__((packed)) rplidar_ans_header_t;

/* 雷达设备信息结构体 */
typedef struct {
    uint8_t model;              // 型号
    uint16_t firmware_version;  // 固件版本
    uint8_t hardware_version;   // 硬件版本
    uint8_t serialnum[16];      // 序列号
} __attribute__((packed)) rplidar_response_device_info_t;

/* 雷达健康状态结构体 */
typedef struct {
    uint8_t status;             // 状态
    uint16_t error_code;        // 错误代码
} __attribute__((packed)) rplidar_response_device_health_t;

/* 扫描数据点结构体 */
typedef struct {
    uint8_t quality;            // 信号质量 (0-63)
    uint16_t angle_q6_checkbit; // 角度 (14位) + 检查位 (1位) + 起始位 (1位)
    uint16_t distance_q2;       // 距离 (毫米)
} __attribute__((packed)) rplidar_response_measurement_node_t;

/* 处理后的扫描点数据 */
typedef struct {
    float angle;                // 角度 (度)
    float distance;             // 距离 (米)
    uint8_t quality;            // 信号质量
    uint8_t start_flag;         // 起始标志
} rplidar_scan_point_t;

/* 雷达接收状态 */
typedef enum {
    RPLIDAR_STATE_IDLE,
    RPLIDAR_STATE_WAIT_HEADER,
    RPLIDAR_STATE_WAIT_DATA,
    RPLIDAR_STATE_DATA_READY
} rplidar_rx_state_t;

/* 雷达接收缓冲区 */
typedef struct {
    uint8_t buffer[RPLIDAR_RX_BUFFER_SIZE];
    uint16_t write_index;
    uint16_t read_index;
    rplidar_rx_state_t state;
    rplidar_ans_header_t current_header;
    uint16_t expected_size;
    uint32_t last_byte_time;
} rplidar_rx_buffer_t;

/* 雷达控制结构体 */
typedef struct {
    uint8_t is_scanning;        // 是否正在扫描
    uint8_t motor_running;      // 电机是否运行
    uint8_t health_status;      // 健康状态
    uint32_t scan_count;        // 扫描计数
    rplidar_response_device_info_t device_info;
} rplidar_status_t;

/* 外部变量声明 */
extern rplidar_rx_buffer_t rplidar_rx_buf;
extern rplidar_status_t rplidar_status;
extern osMessageQueueId_t lidarDataQueueHandle;

/* 函数声明 */

/* 初始化和控制函数 */
void RPLidar_Init(void);
void RPLidar_Reset(void);
void RPLidar_StartScan(void);
void RPLidar_StopScan(void);
void RPLidar_StartMotor(void);
void RPLidar_StopMotor(void);

/* 命令发送函数 */
void RPLidar_SendCommand(uint8_t cmd, uint8_t *payload, uint8_t payload_size);
void RPLidar_GetDeviceInfo(void);
void RPLidar_GetDeviceHealth(void);

/* 数据接收和处理函数 */
void RPLidar_ProcessRxByte(uint8_t byte);
uint8_t RPLidar_ParseResponse(void);
uint8_t RPLidar_ParseScanData(rplidar_scan_point_t *points, uint16_t *point_count);

/* 数据转换函数 */
float RPLidar_GetAngle(uint16_t angle_q6);
float RPLidar_GetDistance(uint16_t distance_q2);
uint8_t RPLidar_GetQuality(uint8_t quality_byte);

/* 上位机通信函数 */
void RPLidar_SendScanDataToPC(rplidar_scan_point_t *points, uint16_t point_count);
void RPLidar_SendStatusToPC(void);

/* 校验函数 */
uint8_t RPLidar_CalcChecksum(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __RPLIDAR_H */
