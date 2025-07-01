/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : comm_protocol.c
  * @brief          : 通信协议实现与蓝牙通信管理
  ******************************************************************************
  */
/* USER CODE END Header */

#include "comm_protocol.h"
#include "encoder.h"
#include "pid.h"
#include "rplidar.h"

/* 外部变量定义 */
RxBuffer_t bt_rx_buffer = {0};
RxBuffer_t lidar_rx_buffer = {0};

/* 外部变量引用 */
extern UART_HandleTypeDef huart1;  // 雷达串口
extern UART_HandleTypeDef huart3;  // 蓝牙串口
extern osMessageQueueId_t btCommQueueHandle;
extern osMessageQueueId_t lidarCommQueueHandle;
extern osMessageQueueId_t cmdQueueHandle;
extern osMutexId_t UART_MutexHandle;

/* 全局变量 */
static uint32_t heartbeat_counter = 0;
static MotorCtrlData_t current_motor_cmd = {0};

/**
 * @brief CRC16校验计算 (CCITT标准)
 */
uint16_t CRC16_Calculate(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    
    while (length--) {
        crc ^= *data++ << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief 通信帧打包
 */
uint8_t CommFrame_Pack(CommFrame_t *frame, uint8_t *buffer)
{
    uint16_t index = 0;
    uint16_t crc_data_len;
    
    // 帧头
    buffer[index++] = (frame->header >> 8) & 0xFF;
    buffer[index++] = frame->header & 0xFF;
    
    // 设备ID
    buffer[index++] = frame->device_id;
    
    // 命令类型
    buffer[index++] = frame->cmd_type;
    
    // 数据长度
    buffer[index++] = frame->data_length;
    
    // 数据
    if (frame->data_length > 0 && frame->data_length <= MAX_DATA_LENGTH) {
        memcpy(&buffer[index], frame->data, frame->data_length);
        index += frame->data_length;
    }
    
    // 计算CRC (从设备ID到数据结束)
    crc_data_len = 3 + frame->data_length; // device_id + cmd_type + data_length + data
    frame->crc = CRC16_Calculate(&buffer[2], crc_data_len);
    
    // CRC
    buffer[index++] = (frame->crc >> 8) & 0xFF;
    buffer[index++] = frame->crc & 0xFF;
    
    // 帧尾
    buffer[index++] = (frame->tail >> 8) & 0xFF;
    buffer[index++] = frame->tail & 0xFF;
    
    return index; // 返回总长度
}

/**
 * @brief 通信帧解包
 */
uint8_t CommFrame_Unpack(uint8_t *buffer, uint16_t length, CommFrame_t *frame)
{
    uint16_t index = 0;
    uint16_t calc_crc;
    uint16_t crc_data_len;
    
    if (length < FRAME_MIN_LENGTH) {
        return 0; // 长度不够
    }
    
    // 帧头
    frame->header = (buffer[index] << 8) | buffer[index + 1];
    index += 2;
    
    if (frame->header != FRAME_HEADER) {
        return 0; // 帧头错误
    }
    
    // 设备ID
    frame->device_id = buffer[index++];
    
    // 命令类型
    frame->cmd_type = buffer[index++];
    
    // 数据长度
    frame->data_length = buffer[index++];
    
    if (frame->data_length > MAX_DATA_LENGTH) {
        return 0; // 数据长度超限
    }
    
    if (length < (FRAME_MIN_LENGTH + frame->data_length)) {
        return 0; // 总长度不够
    }
    
    // 数据
    if (frame->data_length > 0) {
        memcpy(frame->data, &buffer[index], frame->data_length);
        index += frame->data_length;
    }
    
    // CRC
    frame->crc = (buffer[index] << 8) | buffer[index + 1];
    index += 2;
    
    // 验证CRC
    crc_data_len = 3 + frame->data_length;
    calc_crc = CRC16_Calculate(&buffer[2], crc_data_len);
    if (calc_crc != frame->crc) {
        return 0; // CRC校验失败
    }
    
    // 帧尾
    frame->tail = (buffer[index] << 8) | buffer[index + 1];
    index += 2;
    
    if (frame->tail != FRAME_TAIL) {
        return 0; // 帧尾错误
    }
    
    return 1; // 解包成功
}

/**
 * @brief 通信帧验证
 */
uint8_t CommFrame_Validate(CommFrame_t *frame)
{
    if (frame->header != FRAME_HEADER) return 0;
    if (frame->tail != FRAME_TAIL) return 0;
    if (frame->data_length > MAX_DATA_LENGTH) return 0;
    return 1;
}

/**
 * @brief 蓝牙发送帧
 */
void BT_SendFrame(uint8_t device_id, uint8_t cmd_type, uint8_t *data, uint8_t length)
{
    CommFrame_t frame;
    uint8_t tx_buffer[128];
    uint8_t frame_length;
    
    // 构建帧
    frame.header = FRAME_HEADER;
    frame.device_id = device_id;
    frame.cmd_type = cmd_type;
    frame.data_length = length;
    frame.tail = FRAME_TAIL;
    
    if (length > 0 && data != NULL) {
        memcpy(frame.data, data, length);
    }
    
    // 打包
    frame_length = CommFrame_Pack(&frame, tx_buffer);
    
    // 发送
    if (osMutexAcquire(UART_MutexHandle, 100) == osOK) {
        HAL_UART_Transmit(&huart3, tx_buffer, frame_length, 1000);
        osMutexRelease(UART_MutexHandle);
    }
}

/**
 * @brief 雷达发送帧
 */
void Lidar_SendFrame(uint8_t device_id, uint8_t cmd_type, uint8_t *data, uint8_t length)
{
    CommFrame_t frame;
    uint8_t tx_buffer[128];
    uint8_t frame_length;
    
    // 构建帧
    frame.header = FRAME_HEADER;
    frame.device_id = device_id;
    frame.cmd_type = cmd_type;
    frame.data_length = length;
    frame.tail = FRAME_TAIL;
    
    if (length > 0 && data != NULL) {
        memcpy(frame.data, data, length);
    }
    
    // 打包
    frame_length = CommFrame_Pack(&frame, tx_buffer);
    
    // 发送
    if (osMutexAcquire(UART_MutexHandle, 100) == osOK) {
        HAL_UART_Transmit(&huart1, tx_buffer, frame_length, 1000);
        osMutexRelease(UART_MutexHandle);
    }
}

/**
 * @brief 蓝牙接收数据处理
 */
void BT_ProcessRxData(uint8_t data)
{
    uint32_t current_time = HAL_GetTick();
    
    // 超时检测(100ms)
    if ((current_time - bt_rx_buffer.last_byte_time) > 100) {
        bt_rx_buffer.write_index = 0;
        bt_rx_buffer.state = COMM_IDLE;
    }
    
    bt_rx_buffer.last_byte_time = current_time;
    
    // 缓冲区防溢出
    if (bt_rx_buffer.write_index >= sizeof(bt_rx_buffer.buffer)) {
        bt_rx_buffer.write_index = 0;
        bt_rx_buffer.state = COMM_IDLE;
    }
    
    bt_rx_buffer.buffer[bt_rx_buffer.write_index++] = data;
    
    // 帧检测
    if (bt_rx_buffer.write_index >= 2) {
        uint16_t header = (bt_rx_buffer.buffer[bt_rx_buffer.write_index-2] << 8) | 
                         bt_rx_buffer.buffer[bt_rx_buffer.write_index-1];
        if (header == FRAME_HEADER && bt_rx_buffer.state == COMM_IDLE) {
            bt_rx_buffer.state = COMM_RECEIVING;
            // 将帧头移到缓冲区开始
            bt_rx_buffer.buffer[0] = bt_rx_buffer.buffer[bt_rx_buffer.write_index-2];
            bt_rx_buffer.buffer[1] = bt_rx_buffer.buffer[bt_rx_buffer.write_index-1];
            bt_rx_buffer.write_index = 2;
        }
    }
    
    // 帧尾检测
    if (bt_rx_buffer.write_index >= FRAME_MIN_LENGTH && bt_rx_buffer.state == COMM_RECEIVING) {
        uint16_t tail = (bt_rx_buffer.buffer[bt_rx_buffer.write_index-2] << 8) | 
                       bt_rx_buffer.buffer[bt_rx_buffer.write_index-1];
        if (tail == FRAME_TAIL) {
            bt_rx_buffer.state = COMM_FRAME_READY;
        }
    }
}

/**
 * @brief 雷达接收数据处理
 */
void Lidar_ProcessRxData(uint8_t data)
{
    uint32_t current_time = HAL_GetTick();
    
    // 超时检测(100ms)
    if ((current_time - lidar_rx_buffer.last_byte_time) > 100) {
        lidar_rx_buffer.write_index = 0;
        lidar_rx_buffer.state = COMM_IDLE;
    }
    
    lidar_rx_buffer.last_byte_time = current_time;
    
    // 缓冲区防溢出
    if (lidar_rx_buffer.write_index >= sizeof(lidar_rx_buffer.buffer)) {
        lidar_rx_buffer.write_index = 0;
        lidar_rx_buffer.state = COMM_IDLE;
    }
    
    lidar_rx_buffer.buffer[lidar_rx_buffer.write_index++] = data;
    
    // 帧检测逻辑同蓝牙
    if (lidar_rx_buffer.write_index >= 2) {
        uint16_t header = (lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-2] << 8) | 
                         lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-1];
        if (header == FRAME_HEADER && lidar_rx_buffer.state == COMM_IDLE) {
            lidar_rx_buffer.state = COMM_RECEIVING;
            lidar_rx_buffer.buffer[0] = lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-2];
            lidar_rx_buffer.buffer[1] = lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-1];
            lidar_rx_buffer.write_index = 2;
        }
    }
    
    if (lidar_rx_buffer.write_index >= FRAME_MIN_LENGTH && lidar_rx_buffer.state == COMM_RECEIVING) {
        uint16_t tail = (lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-2] << 8) | 
                       lidar_rx_buffer.buffer[lidar_rx_buffer.write_index-1];
        if (tail == FRAME_TAIL) {
            lidar_rx_buffer.state = COMM_FRAME_READY;
        }
    }
}

/**
 * @brief 蓝牙帧解析
 */
uint8_t BT_ParseFrame(CommFrame_t *frame)
{
    if (bt_rx_buffer.state != COMM_FRAME_READY) {
        return 0;
    }
    
    uint8_t result = CommFrame_Unpack(bt_rx_buffer.buffer, bt_rx_buffer.write_index, frame);
    
    // 重置接收状态
    bt_rx_buffer.write_index = 0;
    bt_rx_buffer.state = COMM_IDLE;
    
    return result;
}

/**
 * @brief 雷达帧解析
 */
uint8_t Lidar_ParseFrame(CommFrame_t *frame)
{
    if (lidar_rx_buffer.state != COMM_FRAME_READY) {
        return 0;
    }
    
    uint8_t result = CommFrame_Unpack(lidar_rx_buffer.buffer, lidar_rx_buffer.write_index, frame);
    
    // 重置接收状态
    lidar_rx_buffer.write_index = 0;
    lidar_rx_buffer.state = COMM_IDLE;
    
    return result;
}

/**
 * @brief 命令处理路由
 */
void Comm_HandleCommand(CommFrame_t *frame, uint8_t source)
{
    switch (frame->cmd_type) {
        case CMD_HEARTBEAT:
            // 心跳包处理
            heartbeat_counter++;
            break;
            
        case CMD_MOTOR_CTRL:
            // 电机控制命令
            if (frame->data_length == sizeof(MotorCtrlData_t)) {
                memcpy(&current_motor_cmd, frame->data, sizeof(MotorCtrlData_t));
                // 发送到电机控制任务队列
                osMessageQueuePut(cmdQueueHandle, &current_motor_cmd, 0, 0);
            }
            break;
            
        case CMD_GET_PARAM:
            // 参数获取请求
            switch (frame->data[0]) {
                case 0x01: // 电机状态
                    Comm_SendMotorStatus();
                    break;
                case 0x02: // IMU数据
                    Comm_SendIMUData();
                    break;
                case 0x03: // 里程计数据
                    Comm_SendOdomData();
                    break;
            }
            break;
            
        case CMD_SET_PARAM:
            // 参数设置
            // TODO: 实现参数设置逻辑
            break;
            
        case CMD_LIDAR_START_SCAN:
            // 开始雷达扫描
            RPLidar_StartScan();
            if (source == 0) { // 来自蓝牙
                uint8_t ack_data = CMD_LIDAR_START_SCAN;
                BT_SendFrame(DEVICE_STM32, CMD_ACK, &ack_data, 1);
            }
            break;
            
        case CMD_LIDAR_STOP_SCAN:
            // 停止雷达扫描
            RPLidar_StopScan();
            if (source == 0) {
                uint8_t ack_data = CMD_LIDAR_STOP_SCAN;
                BT_SendFrame(DEVICE_STM32, CMD_ACK, &ack_data, 1);
            }
            break;
            
        case CMD_LIDAR_START_MOTOR:
            // 启动雷达电机
            RPLidar_StartMotor();
            if (source == 0) {
                uint8_t ack_data = CMD_LIDAR_START_MOTOR;
                BT_SendFrame(DEVICE_STM32, CMD_ACK, &ack_data, 1);
            }
            break;
            
        case CMD_LIDAR_STOP_MOTOR:
            // 停止雷达电机
            RPLidar_StopMotor();
            if (source == 0) {
                uint8_t ack_data = CMD_LIDAR_STOP_MOTOR;
                BT_SendFrame(DEVICE_STM32, CMD_ACK, &ack_data, 1);
            }
            break;
            
        case CMD_LIDAR_RESET:
            // 雷达复位
            RPLidar_Reset();
            if (source == 0) {
                uint8_t ack_data = CMD_LIDAR_RESET;
                BT_SendFrame(DEVICE_STM32, CMD_ACK, &ack_data, 1);
            }
            break;
            
        case CMD_LIDAR_GET_STATUS:
            // 获取雷达状态
            RPLidar_SendStatusToPC();
            break;
            
        default:
        {
            // 未知命令，发送NACK
            uint8_t nack_data = frame->cmd_type;
            if (source == 0) { // 来自蓝牙
                BT_SendFrame(DEVICE_STM32, CMD_NACK, &nack_data, 1);
            }
            break;
        }
    }
}

/**
 * @brief 发送电机状态
 */
void Comm_SendMotorStatus(void)
{
    MotorStatusData_t status;
    
    // 获取当前电机状态
    status.left_speed = GetMotorSpeed(LEFT);
    status.right_speed = GetMotorSpeed(RIGHT);
    status.left_current = 0;  // TODO: 实现电流检测
    status.right_current = 0; // TODO: 实现电流检测
    status.status = 0x01;     // 正常状态
    
    BT_SendFrame(DEVICE_STM32, CMD_MOTOR_STATUS, (uint8_t*)&status, sizeof(status));
}

/**
 * @brief 发送IMU数据
 */
void Comm_SendIMUData(void)
{
    IMUData_t imu_data = {0};
    
    // TODO: 从IMU任务获取实际数据
    imu_data.accel_x = 0.0f;
    imu_data.accel_y = 0.0f;
    imu_data.accel_z = 9.8f;
    imu_data.temperature = 25.0f;
    
    BT_SendFrame(DEVICE_STM32, CMD_IMU_DATA, (uint8_t*)&imu_data, sizeof(imu_data));
}

/**
 * @brief 发送里程计数据
 */
void Comm_SendOdomData(void)
{
    OdomData_t odom_data = {0};
    
    // TODO: 从里程计任务获取实际数据
    odom_data.x = 0.0f;
    odom_data.y = 0.0f;
    odom_data.theta = 0.0f;
    odom_data.timestamp = HAL_GetTick();
    
    BT_SendFrame(DEVICE_STM32, CMD_ODOM_DATA, (uint8_t*)&odom_data, sizeof(odom_data));
}

/**
 * @brief 发送心跳包
 */
void Comm_SendHeartbeat(void)
{
    uint32_t system_time = HAL_GetTick();
    BT_SendFrame(DEVICE_STM32, CMD_HEARTBEAT, (uint8_t*)&system_time, sizeof(system_time));
}

/* 接收数据临时变量 */
static uint8_t bt_rx_byte;
static uint8_t lidar_rx_byte;

/**
 * @brief 通信初始化
 */
void Comm_Init(void)
{
    // 初始化接收缓冲区
    memset(&bt_rx_buffer, 0, sizeof(bt_rx_buffer));
    memset(&lidar_rx_buffer, 0, sizeof(lidar_rx_buffer));
    
    // 启动UART中断接收
    HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    HAL_UART_Receive_IT(&huart1, &lidar_rx_byte, 1);
}

/**
 * @brief UART接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        // 蓝牙数据接收
        BT_ProcessRxData(bt_rx_byte);
        
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    }
    else if (huart->Instance == USART1) {
        // 雷达数据接收
        RPLidar_ProcessRxByte(lidar_rx_byte);
        
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &lidar_rx_byte, 1);
    }
}
