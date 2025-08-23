/*UARTProtocol.h*/
#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdint.h>
#include "Nailong.h"  // 包含STAT结构体定义

// 协议配置
#define PROTOCOL_HEADER         0xAA55
#define PROTOCOL_FOOTER         0x0D0A
#define HEARTBEAT_INTERVAL_MS   300
#define HEARTBEAT_TIMEOUT_MS    500
#define MAX_FRAME_LENGTH        256

// 命令字定义
typedef enum {
    CMD_MOTION_CONTROL = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_HEARTBEAT_ACK = 0x03,
    CMD_STATUS_FEEDBACK = 0x04,  // 状态反馈
    CMD_ERROR_REPORT = 0x05,
    CMD_REQUEST_STATUS = 0x06,    // 状态请求命令
    CMD_SERVO_CONTROL = 0x07,     // 舵机控制命令
		CMD_REQUEST_MOTOR_SPEED = 0x08,  //电机转速请求命令 由于状态请求命令太长，速度慢，增加新指令
		CMD_MOTOR_SPEED_FEEDBACK = 0x09  //电机转速反馈
} ProtocolCmd;

/*
#pragma pack(1)
typedef struct STAT//Robot status infomation
{
	bool Ready;
	bool motorEN;
	bool servoEN;
	uint8_t onlineservoid[10];//online servo id     0xFF means not exist
	uint8_t req_servo[10];//request servo angle (°)
	uint8_t servo[10];//physical servo angle (°)
	uint8_t speedLimit;//max speed(RPM)
	float req_speedL;//request speed left(RPM)
	float req_speedR;//right
	float speedL;//physical  speed left(RPM)
	float speedR;//right
	float batteryVoltage;//V
	float motorCurrentL;//Current consumption for MotorDriver L (A)
	float motorCurrentR;//Current consumption for MotorDriver R (A)
	float motorCurrent;//Current consumption for MotorDriver L+R (A)
  uint8_t   CPU;//MCU's CPU usage rate(%)
  uint8_t   RAM;//MCU's Stack usage rate(%)
	uint8_t error_flags;
}STAT;
#pragma pack()
*/


// 接收状态机
typedef enum {
    FRAME_HEADER1,
    FRAME_HEADER2,
    FRAME_LENGTH,
    FRAME_CMD,
    FRAME_DATA,
    FRAME_CRC,
    FRAME_TAIL1,
    FRAME_TAIL2
} RxState;

// 发送消息结构
typedef struct {
    uint8_t *data;
    uint16_t len;
} uart_msg;
 
// 运动控制结构
#pragma pack(push, 1)
typedef struct {
    float target_speedL; 
    float target_speedR; 
} MotionCmd;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t servo_id;
    int16_t angle;
} ServoCmd;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float speedL;   // 左电机实际速度 (RPM)
    float speedR;   // 右电机实际速度 (RPM)
} MotorSpeedFeedback;
#pragma pack(pop) 

// 外部声明
extern osMessageQueueId_t xUartRxQueue;
extern osMessageQueueId_t xUartTxQueue;
extern osSemaphoreId_t xHeartbeatAckSemaphore;
// 外部声明串口句柄
extern UART_HandleTypeDef huart1;

// 函数声明
void UART_Protocol_Init(void);
void UART_Protocol_ReceiveTask(void *argument);
void UART_Protocol_HeartbeatTask(void *argument);
void UART_Protocol_ReconnectTask(void *argument);
void send_protocol_frame(ProtocolCmd cmd, const void *data, uint8_t data_len);
void process_motion_cmd(const MotionCmd *cmd);
void send_status_feedback(void);
void handle_protocol_error(uint8_t error_code);
void process_servo_cmd(const ServoCmd *cmd); // 舵机处理函数
void send_speed_feedback(void);
void UART_Protocol_SendTask(void *argument); // 发送任务

#endif /* UART_PROTOCOL_H */
