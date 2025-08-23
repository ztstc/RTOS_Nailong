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

// ���к��ź���
osMessageQueueId_t xUartRxQueue = NULL;
osMessageQueueId_t xUartTxQueue = NULL;
osSemaphoreId_t xHeartbeatAckSemaphore = NULL;

// ����״̬����
static RxState rx_state = FRAME_HEADER1;
static uint8_t rx_buffer[MAX_FRAME_LENGTH];
static uint8_t rx_index = 0;
static uint8_t expected_length = 0;
static uint8_t current_cmd = 0;
static uint8_t calculated_crc = 0;

// CRC8���� (����ʽ0x07)
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

// �������ջ�����
static uint8_t uart_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
				HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        // �����յ����ֽڷ������
				//HAL_UART_Transmit(&huart1,&uart_rx_byte, 1, 100);
        osMessageQueuePut(xUartRxQueue, &uart_rx_byte, 0, 0);
				
				//printf("%c",uart_rx_byte);
        // ������������
    }
}

// ��ʼ��Э��ģ��
void UART_Protocol_Init(void) {
    // ���������ֽڶ���
    xUartRxQueue = osMessageQueueNew(128, sizeof(uint8_t), NULL);
    
    // ����������Ӧ�ź���
    xHeartbeatAckSemaphore = osSemaphoreNew(1, 0, NULL);
    
    // ����Э������
    const osThreadAttr_t protocolTask_attributes = {
        .name = "ProtocolTask",
        .stack_size = 512,
        .priority = osPriorityHigh,
    };
    osThreadNew(UART_Protocol_ReceiveTask, NULL, &protocolTask_attributes);
    
    // ������������
    const osThreadAttr_t heartbeatTask_attributes = {
        .name = "HeartbeatTask",
        .stack_size = 512,
        .priority = osPriorityNormal,
    };
    osThreadNew(UART_Protocol_HeartbeatTask, NULL, &heartbeatTask_attributes);
    
    // �������Ͷ���
    xUartTxQueue = osMessageQueueNew(10, sizeof(uart_msg), NULL);
    
    // ������������
    const osThreadAttr_t sendTask_attributes = {
        .name = "SendTask",
        .stack_size = 512,
        .priority = osPriorityHigh,
    };
    osThreadNew(UART_Protocol_SendTask, NULL, &sendTask_attributes);
    
    // ������һ�ν���
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

// Э�鷢������
void UART_Protocol_SendTask(void *argument) {
    uart_msg msg;
    
    for (;;) {
        if (osMessageQueueGet(xUartTxQueue, &msg, NULL, osWaitForever) == osOK) {
            // ����ʽ���ͣ��򵥿ɿ���
            HAL_UART_Transmit(&huart1, msg.data, msg.len, 100);
            
            // ������ɺ��ͷ��ڴ�
            vPortFree(msg.data);
        }
    }
}

// Э���������
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
                    
                    if (expected_length <= MAX_FRAME_LENGTH - 4) { // ��ȥ���CRC��֡β�Ŀռ�
                        rx_index = 0;
                        rx_state = FRAME_CMD;
                    } else {
                        handle_protocol_error(0x02); // ���ȴ���
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
                        handle_protocol_error(0x01); // CRC����
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
                        // ����֡����
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
                                // ����������Ӧ
                                send_protocol_frame(CMD_HEARTBEAT_ACK, NULL, 0);
                                // ֪ͨ��������
                                osSemaphoreRelease(xHeartbeatAckSemaphore);
                                break;
                                
                            case CMD_REQUEST_STATUS:
                                send_status_feedback();
                                break;
														
														case CMD_REQUEST_MOTOR_SPEED:
																send_speed_feedback();
																break;
                            default:
                                handle_protocol_error(0x03); // δ֪����
                                break;
                        }
                    }
                    rx_state = FRAME_HEADER1;
                    break;
            }
        }
    }
}

// ��������������
void process_servo_cmd(const ServoCmd *cmd) {
    // �����ID�Ƿ���Ч
    if (cmd->servo_id >= 10) {
        handle_protocol_error(0x04); // ���ID����
        return;
    }
    
    // ��������Ƕ�
    stat.req_servo[cmd->servo_id] = cmd->angle;
    
    // ���ƶ��
    SetServoAngle(cmd->servo_id, cmd->angle, 0); // Ĭ������ٶ�
    // ����״̬����
    //send_status_feedback();
}

// ��������
void UART_Protocol_HeartbeatTask(void *argument) {
    const uint32_t heartbeatInterval = osKernelGetTickFreq() * HEARTBEAT_INTERVAL_MS / 1000;
    uint8_t missedBeats = 0;
    
    for (;;) {
        // ��������
        send_protocol_frame(CMD_HEARTBEAT, NULL, 0);
        
        // �ȴ�������Ӧ
        if (osSemaphoreAcquire(xHeartbeatAckSemaphore, HEARTBEAT_TIMEOUT_MS) == osOK) {
            missedBeats = 0; // �յ���Ӧ�����ü�����
						//printf("OK\r\n");
        } else {
						missedBeats++;
						//printf("missedBeats! %d",missedBeats);
            if (missedBeats >= 3) {
                // ������������
                UART_Protocol_ReconnectTask(NULL);
                missedBeats = 0;
            }
        }
        
        osDelay(heartbeatInterval);
    }
}

// ������������
void UART_Protocol_ReconnectTask(void *argument) {
    EnterSafemode(0x50); // ������� �Ͽ�����
	
    osDelay(100);
    // ��������
    for (;;) {
        // ʹ�þ�̬���������涯̬����
        uint8_t handshake_frame[] = {
            0xAA, 0x55, 
            0x00,                 // ���ݳ���=0
            CMD_HEARTBEAT,         // ������
            0x0E,                 // CRC
            0x0D, 0x0A            // ֡β
        };
        
        // ֱ�ӷ��ͣ����⶯̬���䣩
        HAL_UART_Transmit(&huart1, handshake_frame, sizeof(handshake_frame), 100);
        
        // �ȴ���Ӧ
        if (osSemaphoreAcquire(xHeartbeatAckSemaphore, 500) == osOK) {
            // �����ɹ�
            QuitSafemode();
            return;
        }
        
        // �ȴ�һ��ʱ��������
        osDelay(500);
    }
}

// ����Э��֡
void send_protocol_frame(ProtocolCmd cmd, const void *data, uint8_t data_len) {
    // ����֡���� (header + len + cmd + data + crc + footer)
    uint16_t frame_len = 2 + 1 + 1 + data_len + 1 + 2;
    uint8_t *frame = pvPortMalloc(frame_len);
    
    if (!frame) return;
    
    uint8_t *ptr = frame;
    
    // ֡ͷ
    *ptr++ = 0xAA;
    *ptr++ = 0x55;
    
    // ����
    *ptr++ = data_len;
    
    // ������
    *ptr++ = cmd;
    
    // ������
    if (data_len > 0 && data) {
        memcpy(ptr, data, data_len);
        ptr += data_len;
    }
    
    // ����CRC (�ӳ��ȿ�ʼ�����ݽ���)
    uint8_t crc = 0;
    crc = crc8_update(crc, data_len);
    crc = crc8_update(crc, cmd);
    for (int i = 0; i < data_len; i++) {
        crc = crc8_update(crc, ((uint8_t*)data)[i]);
    }
    *ptr++ = crc;
    
    // ֡β
    *ptr++ = 0x0D;
    *ptr++ = 0x0A;
    
    // ������Ϣ
    uart_msg msg = {
        .data = frame,
        .len = frame_len
    };
    
    // ���͵����У���������
    if (osMessageQueuePut(xUartTxQueue, &msg, 0, 0) != osOK) {
        vPortFree(frame);
    }
}

// �����˶���������
void process_motion_cmd(const MotionCmd *cmd) {
    // ���������ٶȵ�״̬�ṹ��
    stat.req_speedL = cmd->target_speedL;
    stat.req_speedR = cmd->target_speedR;
    
    // ����ָ������
    CANopen_SetTargetSpeed(stat.req_speedL, stat.req_speedR);
    
    // ����״̬����
    //send_status_feedback();
}

// ����״̬����
void send_status_feedback(void) {
    // ����STAT�ṹ��
    send_protocol_frame(CMD_STATUS_FEEDBACK, &stat, sizeof(STAT));
}

//����̵��ٶȷ���
void send_speed_feedback(void){
	MotorSpeedFeedback feedback;
    feedback.speedL = stat.speedL;
    feedback.speedR = stat.speedR;
    send_protocol_frame(CMD_MOTOR_SPEED_FEEDBACK, &feedback, sizeof(MotorSpeedFeedback));
}

// ����Э�����
void handle_protocol_error(uint8_t error_code) {
    // �������߼���������Ҫʵ�֣�
	printf("protocol_error %d \r\n",error_code);
}
