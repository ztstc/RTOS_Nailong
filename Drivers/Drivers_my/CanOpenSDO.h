#ifndef __CANOPENSDO_H
#define __CANOPENSDO_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"  // 添加信号量支持

extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern void Error_Handler(void);
extern int fputc(int ch,FILE *f);


// 定义节点ID
#define NODE_ID 1
#define std_id 0x601;
// SDO命令字节
#define SDO_READ 0x40 //读取 M->S 请求 0 字节
#define SDO_WRITE_1B 0x2F//设置 M->S 请求 1 字节
#define SDO_WRITE_2B 0x2B//设置 M->S 请求 2 字节
#define SDO_WRITE_4B 0x23//设置 M->S 请求 4 字节


// 对象字典索引
#define INDEX_MODE_OF_OPERATION 0x6060//设置模式
#define INDEX_TARGET_VELOCITY 0x60FF //设置目标速度
#define INDEX_TARGET_POSITION 0x607A //设置目标位置
#define INDEX_TARGET_MAXSPEED 0x6081 //设置目标最大速度
#define INDEX_TARGET_TORQUE 0x6071 //设置目标转矩
#define INDEX_CONTROL_WORD 0x6040 //电机使能
#define INDEX_STATUS_WORD 0x6041   // 状态字索引
#define ESTOP_BIT 15               // 急停状态位 (bit15)
#define ENABLE_BITS_MASK 0x000F    // 使能状态掩码 (低4位)
#define ENABLED_STATE 0x0007       // 使能状态值 (0111)

// 初始化CAN接口
void CANopen_Init(void);

// 发送SDO请求
void CANopen_SendSDORequest(uint8_t command, uint16_t index, uint8_t subindex, uint32_t data);

// 接收SDO响应
uint8_t CANopen_ReceiveSDOResponse(uint8_t* response);

// 设置电机速度模式
void CANopen_SetSpeedMode(void);

// 设置电机目标速度
void CANopen_SetTargetSpeed(float speedL, float speedR);

//设置电机限速
void  CANopen_SetSpeedLimit(uint16_t speed);

//位置模式设置左电机最大速度
void CANopen_SetLeftMaxSpeed(int8_t speed);
//位置模式设置右电机最大速度
void CANopen_SetRightMaxSpeed(int8_t speed);

//设置左电机目标位置
void CANopen_SetLeftPosition(int32_t position);
//设置右电机目标位置
void CANopen_SetRightPosition(int32_t position);

//启动相对运动
void CANopen_StartRelative(void);

//启动绝对运动
void CANopen_StartAbsolute(void);

// 启动电机
void CANopen_StartMotor(void);

// 停止电机 电机失能 解轴
void CANopen_StopMotor(void);

//设置力(转）矩模式
void CANopen_SetTorqueMode(void);

//设置左右电机目标转矩（mA/s）
void CANopen_SetTorque(uint32_t left,uint32_t right);

// 设置速度分辨率 (索引2026h 子索引05)
/*设置值范围：1-A
1：速度分辨率 1RPM
2：速度分辨率 0.5RPM
3：速度分辨率 1/3RPM
4：速度分辨率 0.25RPM
5：速度分辨率 0.2RPM
6：速度分辨率 1/6RPM
7：速度分辨率 1/7RPM
8：速度分辨率 0.125RPM
9：速度分辨率 1/9RPM
A：速度分辨率 0.1RPM*/
void CANopen_SetSpeedResolution(uint8_t resolution);

//保存设置
void CANopen_SaveToEEPROM(void);

// 设置左电机加速时间 (单位ms)
void CANopen_SetLeftAccelTime(uint16_t time_ms) ;

// 设置右电机加速时间 (单位ms)
void CANopen_SetRightAccelTime(uint16_t time_ms);

// 设置左电机减速时间 (单位ms)
void CANopen_SetLeftDecelTime(uint16_t time_ms);

// 设置右电机减速时间 (单位ms)
void CANopen_SetRightDecelTime(uint16_t time_ms);

// 同时设置左右电机加减速时间 (单位ms)
void CANopen_SetMotorAccelDecel(uint16_t accel_time, uint16_t decel_time);

//错误中断
void Error_Handler(void);

//心跳报文启动
void CANopen_Heartbeat(void);
	
//接收消息处理
void CANopen_MessageHandler(uint8_t data[8]);

// 更新数据并返回一个数组，包含电压、左右电机总电流、左右电机实际转速、左右电机目标转速、电机最大运行速度
uint8_t CanOpen_UpdateInfo(void);

//检查can队列未发送的消息
uint32_t Checkmailbox(void);


#endif
