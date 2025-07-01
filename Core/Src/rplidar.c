/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : rplidar.c
  * @brief          : 思岚C1激光雷达驱动实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "rplidar.h"
#include "comm_protocol.h"
#include <string.h>
#include <math.h>

/* 外部变量引用 */
extern UART_HandleTypeDef huart1;  // 雷达串口
extern osMutexId_t UART_MutexHandle;

/* 全局变量定义 */
rplidar_rx_buffer_t rplidar_rx_buf = {0};
rplidar_status_t rplidar_status = {0};

/* 扫描数据缓冲区 */
static rplidar_scan_point_t scan_points[RPLIDAR_MAX_SCAN_POINTS];
static uint16_t current_point_count = 0;

/**
 * @brief 雷达初始化
 */
void RPLidar_Init(void)
{
    // 初始化接收缓冲区
    memset(&rplidar_rx_buf, 0, sizeof(rplidar_rx_buf));
    rplidar_rx_buf.state = RPLIDAR_STATE_IDLE;
    
    // 初始化雷达状态
    memset(&rplidar_status, 0, sizeof(rplidar_status));
    
    // 停止雷达
    RPLidar_Reset();
    osDelay(100);
    
    // 获取设备信息
    RPLidar_GetDeviceInfo();
    osDelay(50);
    
    // 获取健康状态
    RPLidar_GetDeviceHealth();
    osDelay(50);
    
    // 启动电机
    RPLidar_StartMotor();
    osDelay(1000); // 等待电机稳定
}

/**
 * @brief 雷达复位
 */
void RPLidar_Reset(void)
{
    RPLidar_SendCommand(RPLIDAR_CMD_RESET, NULL, 0);
    rplidar_status.is_scanning = 0;
    rplidar_status.motor_running = 0;
}

/**
 * @brief 开始扫描
 */
void RPLidar_StartScan(void)
{
    if (!rplidar_status.motor_running) {
        RPLidar_StartMotor();
        osDelay(1000); // 等待电机稳定
    }
    
    RPLidar_SendCommand(RPLIDAR_CMD_SCAN, NULL, 0);
    rplidar_status.is_scanning = 1;
    
    // 重置接收状态
    rplidar_rx_buf.state = RPLIDAR_STATE_WAIT_HEADER;
    rplidar_rx_buf.write_index = 0;
    rplidar_rx_buf.read_index = 0;
}

/**
 * @brief 停止扫描
 */
void RPLidar_StopScan(void)
{
    RPLidar_SendCommand(RPLIDAR_CMD_STOP, NULL, 0);
    rplidar_status.is_scanning = 0;
}

/**
 * @brief 启动电机
 */
void RPLidar_StartMotor(void)
{
    // 思岚雷达通过DTR信号控制电机
    // 这里我们通过GPIO控制，需要根据实际硬件连接修改
    // 假设使用PC0控制电机
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    rplidar_status.motor_running = 1;
}

/**
 * @brief 停止电机
 */
void RPLidar_StopMotor(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    rplidar_status.motor_running = 0;
}

/**
 * @brief 发送命令到雷达
 */
void RPLidar_SendCommand(uint8_t cmd, uint8_t *payload, uint8_t payload_size)
{
    rplidar_cmd_packet_t packet;
    uint8_t total_size;
    
    packet.sync_byte = RPLIDAR_CMD_SYNC_BYTE;
    packet.cmd_flag = cmd;
    packet.size = payload_size;
    
    if (payload_size > 0 && payload != NULL) {
        memcpy(packet.data, payload, payload_size);
        packet.cmd_flag |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }
    
    // 计算校验和
    packet.checksum = 0;
    uint8_t *data = (uint8_t*)&packet;
    for (int i = 0; i < 3 + payload_size; i++) {
        packet.checksum ^= data[i];
    }
    
    total_size = 3 + payload_size + (payload_size > 0 ? 1 : 0);
    
    // 发送命令
    if (osMutexAcquire(UART_MutexHandle, 100) == osOK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)&packet, total_size, 1000);
        osMutexRelease(UART_MutexHandle);
    }
}

/**
 * @brief 获取设备信息
 */
void RPLidar_GetDeviceInfo(void)
{
    RPLidar_SendCommand(RPLIDAR_CMD_GET_INFO, NULL, 0);
}

/**
 * @brief 获取设备健康状态
 */
void RPLidar_GetDeviceHealth(void)
{
    RPLidar_SendCommand(RPLIDAR_CMD_GET_HEALTH, NULL, 0);
}

/**
 * @brief 处理接收到的字节
 */
void RPLidar_ProcessRxByte(uint8_t byte)
{
    uint32_t current_time = HAL_GetTick();
    
    // 超时检测 (500ms)
    if ((current_time - rplidar_rx_buf.last_byte_time) > 500) {
        rplidar_rx_buf.state = RPLIDAR_STATE_IDLE;
        rplidar_rx_buf.write_index = 0;
        rplidar_rx_buf.read_index = 0;
    }
    
    rplidar_rx_buf.last_byte_time = current_time;
    
    // 防止缓冲区溢出
    if (rplidar_rx_buf.write_index >= RPLIDAR_RX_BUFFER_SIZE) {
        rplidar_rx_buf.write_index = 0;
        rplidar_rx_buf.state = RPLIDAR_STATE_IDLE;
    }
    
    rplidar_rx_buf.buffer[rplidar_rx_buf.write_index++] = byte;
    
    switch (rplidar_rx_buf.state) {
        case RPLIDAR_STATE_IDLE:
        case RPLIDAR_STATE_WAIT_HEADER:
            // 查找应答头
            if (rplidar_rx_buf.write_index >= sizeof(rplidar_ans_header_t)) {
                rplidar_ans_header_t *header = (rplidar_ans_header_t*)rplidar_rx_buf.buffer;
                if (header->sync_byte1 == RPLIDAR_ANS_SYNC_BYTE1 && 
                    header->sync_byte2 == RPLIDAR_ANS_SYNC_BYTE2) {
                    // 找到有效头部
                    memcpy(&rplidar_rx_buf.current_header, header, sizeof(rplidar_ans_header_t));
                    rplidar_rx_buf.expected_size = (rplidar_rx_buf.current_header.size_q30_subtype >> 2) & 0x3FFFFFFF;
                    rplidar_rx_buf.state = RPLIDAR_STATE_WAIT_DATA;
                    rplidar_rx_buf.read_index = sizeof(rplidar_ans_header_t);
                }
            }
            break;
            
        case RPLIDAR_STATE_WAIT_DATA:
            // 检查是否接收完整数据
            if ((rplidar_rx_buf.write_index - rplidar_rx_buf.read_index) >= rplidar_rx_buf.expected_size) {
                rplidar_rx_buf.state = RPLIDAR_STATE_DATA_READY;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief 解析雷达应答
 */
uint8_t RPLidar_ParseResponse(void)
{
    if (rplidar_rx_buf.state != RPLIDAR_STATE_DATA_READY) {
        return 0;
    }
    
    uint8_t result = 0;
    uint8_t *data = &rplidar_rx_buf.buffer[rplidar_rx_buf.read_index];
    
    switch (rplidar_rx_buf.current_header.type) {
        case RPLIDAR_ANS_TYPE_DEVINFO:
            // 设备信息应答
            if (rplidar_rx_buf.expected_size >= sizeof(rplidar_response_device_info_t)) {
                memcpy(&rplidar_status.device_info, data, sizeof(rplidar_response_device_info_t));
                result = 1;
            }
            break;
            
        case RPLIDAR_ANS_TYPE_DEVHEALTH:
            // 健康状态应答
            if (rplidar_rx_buf.expected_size >= sizeof(rplidar_response_device_health_t)) {
                rplidar_response_device_health_t *health = (rplidar_response_device_health_t*)data;
                rplidar_status.health_status = health->status;
                result = 1;
            }
            break;
            
        case RPLIDAR_ANS_TYPE_MEASUREMENT:
            // 扫描数据
            result = RPLidar_ParseScanData(scan_points, &current_point_count);
            if (result && current_point_count > 0) {
                // 发送扫描数据到上位机
                RPLidar_SendScanDataToPC(scan_points, current_point_count);
                rplidar_status.scan_count++;
            }
            break;
            
        default:
            break;
    }
    
    // 重置接收状态
    rplidar_rx_buf.state = RPLIDAR_STATE_IDLE;
    rplidar_rx_buf.write_index = 0;
    rplidar_rx_buf.read_index = 0;
    
    return result;
}

/**
 * @brief 解析扫描数据
 */
uint8_t RPLidar_ParseScanData(rplidar_scan_point_t *points, uint16_t *point_count)
{
    if (!points || !point_count) return 0;
    
    *point_count = 0;
    uint8_t *data = &rplidar_rx_buf.buffer[rplidar_rx_buf.read_index];
    uint16_t available_size = rplidar_rx_buf.expected_size;
    
    // 每个测量点5字节
    uint16_t node_count = available_size / sizeof(rplidar_response_measurement_node_t);
    if (node_count > RPLIDAR_MAX_SCAN_POINTS) {
        node_count = RPLIDAR_MAX_SCAN_POINTS;
    }
    
    rplidar_response_measurement_node_t *nodes = (rplidar_response_measurement_node_t*)data;
    
    for (uint16_t i = 0; i < node_count; i++) {
        points[i].quality = RPLidar_GetQuality(nodes[i].quality);
        points[i].angle = RPLidar_GetAngle(nodes[i].angle_q6_checkbit);
        points[i].distance = RPLidar_GetDistance(nodes[i].distance_q2);
        points[i].start_flag = (nodes[i].angle_q6_checkbit & 0x1) ? 1 : 0;
        
        // 过滤无效数据
        if (points[i].quality > 0 && points[i].distance > 0.1f && points[i].distance < 12.0f) {
            (*point_count)++;
        }
    }
    
    return (*point_count > 0) ? 1 : 0;
}

/**
 * @brief 获取角度值
 */
float RPLidar_GetAngle(uint16_t angle_q6)
{
    return ((angle_q6 >> 1) / 64.0f);
}

/**
 * @brief 获取距离值  
 */
float RPLidar_GetDistance(uint16_t distance_q2)
{
    return (distance_q2 / 4000.0f); // 转换为米
}

/**
 * @brief 获取质量值
 */
uint8_t RPLidar_GetQuality(uint8_t quality_byte)
{
    return (quality_byte >> 2);
}

/**
 * @brief 发送扫描数据到上位机
 */
void RPLidar_SendScanDataToPC(rplidar_scan_point_t *points, uint16_t point_count)
{
    if (!points || point_count == 0) return;
    
    // 构建雷达数据包
    typedef struct {
        uint16_t point_count;
        uint32_t timestamp;
        rplidar_scan_point_t points[RPLIDAR_MAX_SCAN_POINTS];
    } lidar_data_packet_t;
    
    static lidar_data_packet_t lidar_packet;
    
    lidar_packet.point_count = point_count;
    lidar_packet.timestamp = HAL_GetTick();
    
    // 只发送有效点
    uint16_t valid_count = 0;
    for (uint16_t i = 0; i < point_count && valid_count < RPLIDAR_MAX_SCAN_POINTS; i++) {
        if (points[i].quality > 0) {
            lidar_packet.points[valid_count] = points[i];
            valid_count++;
        }
    }
    
    lidar_packet.point_count = valid_count;
    
    // 计算实际数据大小
    uint16_t data_size = sizeof(uint16_t) + sizeof(uint32_t) + 
                        valid_count * sizeof(rplidar_scan_point_t);
    
    // 通过蓝牙发送到上位机
    BT_SendFrame(DEVICE_STM32, CMD_LIDAR_DATA, (uint8_t*)&lidar_packet, data_size);
}

/**
 * @brief 发送雷达状态到上位机
 */
void RPLidar_SendStatusToPC(void)
{
    typedef struct {
        uint8_t is_scanning;
        uint8_t motor_running;
        uint8_t health_status;
        uint32_t scan_count;
        uint8_t model;
        uint16_t firmware_version;
    } lidar_status_packet_t;
    
    lidar_status_packet_t status_packet;
    
    status_packet.is_scanning = rplidar_status.is_scanning;
    status_packet.motor_running = rplidar_status.motor_running;
    status_packet.health_status = rplidar_status.health_status;
    status_packet.scan_count = rplidar_status.scan_count;
    status_packet.model = rplidar_status.device_info.model;
    status_packet.firmware_version = rplidar_status.device_info.firmware_version;
    
    BT_SendFrame(DEVICE_STM32, CMD_SYSTEM_INFO, (uint8_t*)&status_packet, sizeof(status_packet));
}

/**
 * @brief 计算校验和
 */
uint8_t RPLidar_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
