/*UARTProtocol.c*/
#include "UARTProtocol.h"
#include "servo.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "Nailong.h"
#include "stm32f1xx_hal.h"
#include "CanOpenSDO.h"

extern UART_HandleTypeDef huart1;
extern void MX_USART1_UART_Init(void);

// 队列和信号量
osMessageQueueId_t xUartRxQueue = NULL;
osMessageQueueId_t xUartTxQueue = NULL;
osSemaphoreId_t xHeartbeatAckSemaphore = NULL;

// 接收状态变量
static RxState rx_state = FRAME_HEADER1;
static uint8_t rx_buffer[MAX_FRAME_LENGTH];
static uint8_t rx_index = 0;
static uint8_t expected_length = 0;
static uint8_t current_cmd = 0;
static uint8_t calculated_crc = 0;

// CRC8计算 (多项式0x07)
uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

// 声明接收缓冲区
static uint8_t uart_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
				HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        // 将接收到的字节放入队列
				//HAL_UART_Transmit(&huart1,&uart_rx_byte, 1, 100);
        osMessageQueuePut(xUartRxQueue, &uart_rx_byte, 0, 0);
				
				//printf("%c",uart_rx_byte);
        // 重新启动接收
    }
}

// 初始化协议模块
void UART_Protocol_Init(void) {
    // 创建接收字节队列
    xUartRxQueue = osMessageQueueNew(128, sizeof(uint8_t), NULL);
    
    // 创建心跳响应信号量
    xHeartbeatAckSemaphore = osSemaphoreNew(1, 0, NULL);
    
    // 创建协议任务
    const osThreadAttr_t protocolTask_attributes = {
        .name = "ProtocolTask",
        .stack_size = 512,
        .priority = osPriorityHigh,
    };
    osThreadNew(UART_Protocol_ReceiveTask, NULL, &protocolTask_attributes);
    
    // 创建心跳任务
    const osThreadAttr_t heartbeatTask_attributes = {
        .name = "HeartbeatTask",
        .stack_size = 512,
        .priority = osPriorityNormal,
    };
    osThreadNew(UART_Protocol_HeartbeatTask, NULL, &heartbeatTask_attributes);
    
    // 创建发送队列
    xUartTxQueue = osMessageQueueNew(10, sizeof(uart_msg), NULL);
    
    // 创建发送任务
    const osThreadAttr_t sendTask_attributes = {
        .name = "SendTask",
        .stack_size = 512,
        .priority = osPriorityHigh,
    };
    osThreadNew(UART_Protocol_SendTask, NULL, &sendTask_attributes);
    
    // 启动第一次接收
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

// 协议发送任务
void UART_Protocol_SendTask(void *argument) {
    uart_msg msg;
    
    for (;;) {
        if (osMessageQueueGet(xUartTxQueue, &msg, NULL, osWaitForever) == osOK) {
            // 阻塞式发送（简单可靠）
            HAL_UART_Transmit(&huart1, msg.data, msg.len, 100);
            
            // 发送完成后释放内存
            vPortFree(msg.data);
        }
    }
}

// 协议接收任务
void UART_Protocol_ReceiveTask(void *argument) {
    uint8_t rx_byte;
    
    for (;;) {
        if (osMessageQueueGet(xUartRxQueue, &rx_byte, NULL, osWaitForever) == osOK) {
            switch (rx_state) {
                case FRAME_HEADER1:
                    if (rx_byte == 0xAA) {
                        rx_state = FRAME_HEADER2;
                    }
                    break;
                    
                case FRAME_HEADER2:
                    if (rx_byte == 0x55) {
                        rx_state = FRAME_LENGTH;
                    } else {
                        rx_state = FRAME_HEADER1;
                    }
                    break;
                    
                case FRAME_LENGTH:
                    expected_length = rx_byte;
                    calculated_crc = crc8_update(0, rx_byte);
                    
                    if (expected_length <= MAX_FRAME_LENGTH - 4) { // 减去命令、CRC和帧尾的空间
                        rx_index = 0;
                        rx_state = FRAME_CMD;
                    } else {
                        handle_protocol_error(0x02); // 长度错误
                        rx_state = FRAME_HEADER1;
                    }
                    break;
                    
                case FRAME_CMD:
                    current_cmd = rx_byte;
                    calculated_crc = crc8_update(calculated_crc, rx_byte);
                    
                    if (expected_length > 0) {
                        rx_state = FRAME_DATA;
                    } else {
                        rx_state = FRAME_CRC;
                    }
                    break;
                    
                case FRAME_DATA:
                    rx_buffer[rx_index++] = rx_byte;
                    calculated_crc = crc8_update(calculated_crc, rx_byte);
                    
                    if (rx_index >= expected_length) {
                        rx_state = FRAME_CRC;
                    }
                    break;
                    
                case FRAME_CRC:
                    if (rx_byte == calculated_crc) {
                        rx_state = FRAME_TAIL1;
                    } else {
                        handle_protocol_error(0x01); // CRC错误
                        rx_state = FRAME_HEADER1;
                    }
                    break;
                    
                case FRAME_TAIL1:
                    if (rx_byte == 0x0D) {
                        rx_state = FRAME_TAIL2;
                    } else {
                        rx_state = FRAME_HEADER1;
                    }
                    break;
                    
                case FRAME_TAIL2:
                    if (rx_byte == 0x0A) {
                        // 完整帧处理
                        switch (current_cmd) {
                            case CMD_MOTION_CONTROL:
                                if (expected_length == sizeof(MotionCmd)) {
                                    process_motion_cmd((MotionCmd*)rx_buffer);
                                }
                                break;
                                
                            case CMD_SERVO_CONTROL:
                                if (expected_length == sizeof(ServoCmd)) {
                                    process_servo_cmd((ServoCmd*)rx_buffer);
                                }
                                break;
                                
                            case CMD_HEARTBEAT:
                                // 发送心跳响应
                                send_protocol_frame(CMD_HEARTBEAT_ACK, NULL, 0);
                                // 通知心跳任务
                                osSemaphoreRelease(xHeartbeatAckSemaphore);
                                break;
                                
                            case CMD_REQUEST_STATUS:
                                send_status_feedback();
                                break;
														
														case CMD_REQUEST_MOTOR_SPEED:
																send_speed_feedback();
																break;
                            default:
                                handle_protocol_error(0x03); // 未知命令
                                break;
                        }
                    }
                    rx_state = FRAME_HEADER1;
                    break;
            }
        }
    }
}

// 处理舵机控制命令
void process_servo_cmd(const ServoCmd *cmd) {
    // 检查舵机ID是否有效
    if (cmd->servo_id >= 10) {
        handle_protocol_error(0x04); // 舵机ID错误
        return;
    }
    
    // 更新请求角度
    stat.req_servo[cmd->servo_id] = cmd->angle;
    
    // 控制舵机
    SetServoAngle(cmd->servo_id, cmd->angle, 0); // 默认最快速度
    // 发送状态反馈
    //send_status_feedback();
}

// 心跳任务
void UART_Protocol_HeartbeatTask(void *argument) {
    const uint32_t heartbeatInterval = osKernelGetTickFreq() * HEARTBEAT_INTERVAL_MS / 1000;
    uint8_t missedBeats = 0;
    
    for (;;) {
        // 发送心跳
        send_protocol_frame(CMD_HEARTBEAT, NULL, 0);
        
        // 等待心跳响应
        if (osSemaphoreAcquire(xHeartbeatAckSemaphore, HEARTBEAT_TIMEOUT_MS) == osOK) {
            missedBeats = 0; // 收到响应，重置计数器
						//printf("OK\r\n");
        } else {
						missedBeats++;
						//printf("missedBeats! %d",missedBeats);
            if (missedBeats >= 3) {
                // 触发断线重连
                UART_Protocol_ReconnectTask(NULL);
                missedBeats = 0;
            }
        }
        
        osDelay(heartbeatInterval);
    }
}

// 断线重连任务
void UART_Protocol_ReconnectTask(void *argument) {
    EnterSafemode(0x50); // 错误代码 断开连接
	
    osDelay(100);
    // 尝试重连
    for (;;) {
        // 使用静态缓冲区代替动态分配
        uint8_t handshake_frame[] = {
            0xAA, 0x55, 
            0x00,                 // 数据长度=0
            CMD_HEARTBEAT,         // 命令字
            0x0E,                 // CRC
            0x0D, 0x0A            // 帧尾
        };
        
        // 直接发送（避免动态分配）
        HAL_UART_Transmit(&huart1, handshake_frame, sizeof(handshake_frame), 100);
        
        // 等待响应
        if (osSemaphoreAcquire(xHeartbeatAckSemaphore, 500) == osOK) {
            // 重连成功
            QuitSafemode();
            return;
        }
        
        // 等待一段时间再重试
        osDelay(500);
    }
}

// 发送协议帧
void send_protocol_frame(ProtocolCmd cmd, const void *data, uint8_t data_len) {
    // 计算帧长度 (header + len + cmd + data + crc + footer)
    uint16_t frame_len = 2 + 1 + 1 + data_len + 1 + 2;
    uint8_t *frame = pvPortMalloc(frame_len);
    
    if (!frame) return;
    
    uint8_t *ptr = frame;
    
    // 帧头
    *ptr++ = 0xAA;
    *ptr++ = 0x55;
    
    // 长度
    *ptr++ = data_len;
    
    // 命令字
    *ptr++ = cmd;
    
    // 数据域
    if (data_len > 0 && data) {
        memcpy(ptr, data, data_len);
        ptr += data_len;
    }
    
    // 计算CRC (从长度开始到数据结束)
    uint8_t crc = 0;
    crc = crc8_update(crc, data_len);
    crc = crc8_update(crc, cmd);
    for (int i = 0; i < data_len; i++) {
        crc = crc8_update(crc, ((uint8_t*)data)[i]);
    }
    *ptr++ = crc;
    
    // 帧尾
    *ptr++ = 0x0D;
    *ptr++ = 0x0A;
    
    // 发送消息
    uart_msg msg = {
        .data = frame,
        .len = frame_len
    };
    
    // 发送到队列（非阻塞）
    if (osMessageQueuePut(xUartTxQueue, &msg, 0, 0) != osOK) {
        vPortFree(frame);
    }
}

// 处理运动控制命令
void process_motion_cmd(const MotionCmd *cmd) {
    // 更新请求速度到状态结构体
    stat.req_speedL = cmd->target_speedL;
    stat.req_speedR = cmd->target_speedR;
    
    // 发送指令给电机
    CANopen_SetTargetSpeed(stat.req_speedL, stat.req_speedR);
    
    // 发送状态反馈
    //send_status_feedback();
}

// 发送状态反馈
void send_status_feedback(void) {
    // 发送STAT结构体
    send_protocol_frame(CMD_STATUS_FEEDBACK, &stat, sizeof(STAT));
}

//更简短的速度反馈
void send_speed_feedback(void){
	MotorSpeedFeedback feedback;
    feedback.speedL = stat.speedL;
    feedback.speedR = stat.speedR;
    send_protocol_frame(CMD_MOTOR_SPEED_FEEDBACK, &feedback, sizeof(MotorSpeedFeedback));
}

// 处理协议错误
void handle_protocol_error(uint8_t error_code) {
    // 错误处理逻辑（根据需要实现）
	printf("protocol_error %d \r\n",error_code);
}
