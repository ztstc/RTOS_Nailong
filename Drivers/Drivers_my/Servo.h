
/**********************************************
适用于众灵总线舵机
	* @总线设备基本控制指令表：
	*	1.	#000PID!						//读取ID指令
	*	2.	#000PID001!						//设置ID指令
	*	3.	#000PVER!						//读取版本
	*	4.	#000PBD1!						//设置波特率 默认115200  1:9600 , 2:19200 , 3:38400 , 4:57600 , 5:115200 , 6:128000 7:256000  8:1000000
	*	5.	#000PCLE!						//恢复出厂设置包括ID
	* @总线舵机ZServo控制指令表：
	*	1.	#000P1500T1000!					//舵机角度控制
	*	2.	#000PDST!						//停止
	*	3.	#000PDPT!						//暂停
	*	4.	#000PDCT!						//继续
	*	5.	#000PCSD!						//设置当前值为开机值
	*	6.	#000PCSM!						//开机释放扭力
	*	7.	#000PRAD!						//读取角度
	*	8.	#000PULK!						//释力
	*	9.	#000PULR!						//恢复扭力
	*	10.	#000PSCK!						//设置当前值为1500的偏差值
	*	11.	#000PSCK+050!					//把1500+50作为1500的中间值
	*	12.	#000PSCK-050!					//把1500-50作为1500的中间值
	*	13.	#000PMOD!						//读取模式
	*	14.	#000PMOD1!						//设置模式 舵机模式：2 270 逆时针 1：270 顺 4 180 逆时针 3：180 顺    马达模式：6 360 逆时针 5：360 顺圈 8 360 逆时针 7：360 顺时
	*	15.	#000PRTV!						//读取电压和温度
	*	16.	#000PSTB!						//读取保护值
	*	17.	#000PSTB=60!					//设置保护值 默认60 范围25-80
	*	18.	#000PPAAAIBBB!					//设置KP = AAA, KI = BBB
	*	19.	#000PMIN!						//设置最小值
	*	20.	#000PMAX!						//设置最大值
	*	21.	#000PULM! 						//释力 不带阻力
	*	22.	#000PLN!						//RGB灯开启
	*	23.	#000PLF!						//RGB灯关闭
 **********************************************/
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "log_level.h"
#include "cmsis_os.h"

// 舵机控制指令
typedef enum {
    SERVO_CMD_CONTROL = 0,
    SERVO_CMD_READ_VERSION,
    SERVO_CMD_READ_ID,
    SERVO_CMD_SET_ID,
    SERVO_CMD_RELEASE_TORQUE,
    SERVO_CMD_RESTORE_TORQUE,
    SERVO_CMD_READ_MODE,
    SERVO_CMD_SET_MODE,
    SERVO_CMD_READ_POSITION,
    SERVO_CMD_PAUSE,
    SERVO_CMD_CONTINUE,
    SERVO_CMD_STOP,
    SERVO_CMD_SET_BAUDRATE,
    SERVO_CMD_CALIBRATE,
    SERVO_CMD_SET_START_POSITION,
    SERVO_CMD_REMOVE_START_POSITION,
    SERVO_CMD_RESTORE_START_POSITION,
    SERVO_CMD_SET_MIN_VALUE,
    SERVO_CMD_SET_MAX_VALUE,
    SERVO_CMD_RESET_TO_DEFAULT,
    SERVO_CMD_READ_VOLTAGE_TEMPERATURE
} ServoCommand;

// 全局变量声明
extern float servo_angles[10];  // 存储所有舵机当前角度
//extern osMutexId uart_mutex;    // UART互斥锁

// 初始化舵机库
void ServoBus_Init(UART_HandleTypeDef* huart);

// 启动舵机角度读取任务
void ServoBus_StartReadTask(void);

// 发送控制指令
void ServoBus_SendCommand(uint8_t id, ServoCommand cmd, uint16_t param1, uint16_t param2);

// 读取舵机返回的数据
HAL_StatusTypeDef ServoBus_ReadResponse(uint8_t* buffer, uint16_t size, uint32_t timeout);

// 解析舵机返回消息
float ServoParseMessage(const char* response);

// 设置舵机角度 (0-270°)
void SetServoAngle(uint8_t id, uint16_t angle, uint16_t time);

// 设置舵机PWM (500-2500)
void SetServoPWM(uint8_t id, uint16_t pwm, uint16_t time);

//检测舵机在线情况
uint8_t IsServoOnline(uint8_t id);
uint8_t DiscoverServos(void) ;

//所有舵机解轴
void StopAllServo(void); 

#endif
