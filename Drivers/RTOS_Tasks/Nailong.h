#ifndef NAILONG_H
#define NAILONG_H

/*Nailong.h*/
#include "cmsis_os.h"
#include <stdbool.h>

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

extern STAT stat;

void ButtonLED(bool stat);
void Nailong_Init(void);//初始化机器人 
void NailongTask_Init(void); //RTOS 任务
void QuitSafemode(void);
void EnterSafemode(uint8_t Errcode);//安全模式


#endif // NAILONG_H
