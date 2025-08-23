/*Nailong.c*/
/*奶龙小车基本控制库*/
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "Nailong.h"
#include "OLED.h"
#include "FontImages.h"
#include "CanOpenSDO.h"
#include "Servo.h"
#include "Nailong.h"
#include "btn_task.h"
#include "CPU_Stat.h"
#include "i2c.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

osThreadId_t NailongTaskHandle;

// 定义并初始化全局变量
STAT stat = {
    .Ready = false,
    .motorEN = false,
    .servoEN = false,
		.onlineservoid = {0},
    .req_servo = {0},
    .servo = {0},
    .speedLimit = 0,
    .req_speedL = 0.0f,
    .req_speedR = 0.0f,
    .speedL = 0.0f,
    .speedR = 0.0f,
    .batteryVoltage = 0.0f,
		.motorCurrentL = 0.0f,
		.motorCurrentR = 0.0f,
    .motorCurrent = 0.0f,
    .CPU = 0,
    .RAM = 0,
		.error_flags =0
};

uint8_t previousmenu= 0;
uint8_t menu = 0;
char buffer[64]; 								//转换格式用的缓存区
char txBuffer[128]; 					// 串口发送给上位机的数据

void ButtonLED(bool stat){ //背面Reset按钮的指示灯
	 if(stat){
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	 }else{
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 }
}

// 检查全局中断是否启用 =====测试=====
void Print_Global_IRQ_Status(void) {
    // 使用内部函数读取PRIMASK
    uint32_t primask = __get_PRIMASK();
    
    if (primask & 1) {
        printf("[IRQ] Global interrupts DISABLED\n");
    } else {
        printf("[IRQ] Global interrupts ENABLED\n");
    }
}

void Nailong_Init(void){
	//在调度器启动前不允许使用vTaskDelay和osDelay
	ButtonLED(true);
	HAL_Delay(400);
	ButtonLED(false);
	HAL_Delay(400);
	ButtonLED(true);
	/* OLED初始化 */
	OLED_Init();//此处开始 HAL_Delay 会导致死机
	OLED_ShowImage(0, 0, 128, 64, image_data3); //显示奶龙图标
	OLED_Update(); //更新屏幕画面（不更新不会有改变）
	/* CAN初始化 */
	CANopen_Init();
	HAL_CAN_Start(&hcan);
	CANopen_SetSpeedResolution(0xA);//速度分辨率0.1 设置保存是永久的 不需要再弄了 但这个设置仍然会作用于函数内部的速率换算 
	//CANopen_SaveToEEPROM();//设置保存是永久的 不需要再弄了
	CANopen_SetMotorAccelDecel(500, 500);//加减速时间1500ms
	CANopen_SetSpeedMode();  //设置电机速度模式
	CANopen_StartMotor();		//使能电机	
//	CANopen_SetTargetSpeed(2, 2); //设置目标速度
//	HAL_Delay(100);
	CANopen_SetTargetSpeed(0, 0);
	HAL_Delay(100);
	CANopen_SetSpeedLimit(50);	//设定最大速度

	/*总线舵机初始化*/
	ServoBus_Init(&huart2); //初始化总线舵机
	ServoBus_StartReadTask();
	
	// 发现在线舵机
	//uint8_t servo_count = DiscoverServos(stat.onlineservoid, 10);
	//printf("Found %d online servos\n", servo_count);//不能使用该指令
	//为在线的舵机发送解轴指令
//	for(int i = 0;i<10;i++){
//		if (stat.onlineservoid[i] != 0xFF) {
//			ServoBus_SendCommand(stat.onlineservoid[i],SERVO_CMD_RELEASE_TORQUE,0,0);
//    } 
//	}
	
	for(int pixel=0;pixel<63;pixel+=5){ //神秘小动画
		OLED_ReverseArea(0,pixel,129,5); //逐5行反色 有个进度条动画
		OLED_Update();
	}
	
	OLED_ShowImage(0, 0, 128, 64, image_data1); //切换为主菜单
	OLED_ShowString(0, 3, "Pre", OLED_6X8); //显示初始数据
	OLED_ShowString(38, 3, "0.0V", OLED_6X8);
	OLED_ShowString(72, 3, "0.0A", OLED_6X8);
	OLED_ShowString(5, 33, "0", OLED_6X8);
	OLED_ShowString(5, 51, "0", OLED_6X8);
	OLED_ShowString(75, 33, "0", OLED_6X8);
	OLED_ShowString(75, 51, "0", OLED_6X8);
	OLED_DrawRectangle(106, 3, 18, 6, 1); // 绘制电量条
	OLED_Update();
}

void OLED_UpdateInfo(uint8_t menu){//oled更新数据
		if(menu == 0){
			OLED_ShowImage(0, 0, 128, 64, image_data1);
			if(stat.Ready == true && stat.motorEN == true){
				OLED_ClearArea(0, 3, 38, 8);
				OLED_ShowString(0, 3, "Ready", OLED_6X8);
			}else{
				OLED_ClearArea(0, 3, 38, 8); 
				OLED_ShowString(0, 3, "Stop", OLED_6X8); //急停按钮被按下 或 电机没有使能
			}
			previousmenu=menu;
			
			snprintf(buffer, sizeof(buffer), "%.1fV", stat.batteryVoltage);//电压
			OLED_ClearArea(38,3,30,8);//清空电压数据
			OLED_ClearArea(106,3,18,6);//清空电池条
				/*长度18 25.5-26.5V
			长度15 24.8-25.5V
			长度12 24.0-24.8V
			长度9  23.2-24.0V
			长度6  22.4-23.2V
			长度3  <22.4V (充电)
			 */
//						if(stat.batteryVoltage>=25.5){
//				OLED_DrawRectangle(106,3,18,6,1);//电池条
//			}else if(stat.batteryVoltage>=24.8){
//				OLED_DrawRectangle(106,3,15,6,1);
//			}else if(stat.batteryVoltage>=24){
//				OLED_DrawRectangle(106,3,12,6,1);
//			}else if(stat.batteryVoltage>=23.2){
//				OLED_DrawRectangle(106,3,9,6,1);
//			}else if(stat.batteryVoltage>=22.4){
//				OLED_DrawRectangle(106,3,5,6,1);
//			}else if(stat.batteryVoltage<22.4){
//				//OLED_DrawRectangle(106,3,1,6,1);
//			}
			if (stat.batteryVoltage >= 22.4) {
					float ratio = (stat.batteryVoltage - 22.4f) / (25.5f - 22.4f);
					if (ratio > 1.0f) {
							ratio = 1.0f;
					}
					int length = 1 + (int)(ratio * 17 + 0.5);   // 四舍五入取整，范围1~18
					OLED_DrawRectangle(106, 3, length, 6, 1);
			}
			OLED_ShowString(38, 3, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1fA", stat.motorCurrent);//总电流
			OLED_ClearArea(72,3,30,8);
			OLED_ShowString(72, 3, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.req_speedL);//左电机目标转速
			OLED_ClearArea(5,33,24,8);
			OLED_ShowString(5, 33, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.req_speedR);//右电机目标转速
			OLED_ClearArea(5,51,24,8);
			OLED_ShowString(5, 51, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.speedL);//左电机实际转速
			OLED_ClearArea(75,33,24,8);
			OLED_ShowString(75, 33, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.speedR);//右电机实际转速
			OLED_ClearArea(75,51,24,8);
			OLED_ShowString(75, 51, buffer, OLED_6X8);
			if(stat.req_speedL==0 && stat.req_speedR==0 && stat.motorEN==true){ //P灯亮起
				 if(!OLED_GetPoint(37,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(37,15,13,12);//P高亮(反转显示）
				 }
				 if(OLED_GetPoint(52,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(52,15,13,12);//N变暗(反转显示）
				 }
			}else if(stat.req_speedL==0 && stat.req_speedR==0 && stat.motorEN==false){
					if(OLED_GetPoint(37,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(37,15,13,12);//P变暗(反转显示）
				 }
					if(!OLED_GetPoint(52,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(52,15,13,12);//N高亮(反转显示）
				 }
			 }else{
				 if(OLED_GetPoint(37,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(37,15,13,12);//P变暗(反转显示）
				 }
					if(OLED_GetPoint(52,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(52,15,13,12);//N变暗(反转显示）
				 }
			 }
			if(stat.speedLimit<=25){
				if(!OLED_GetPoint(67,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(67,15,17,12);//D1高亮
				 }
				if(OLED_GetPoint(88,15)){//把别的给灭了
					 OLED_ReverseArea(88,15,17,12);//D2
				 }
				if(OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3
				 }
				//D1档
			}else if(stat.speedLimit<=50){
				if(OLED_GetPoint(67,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(67,15,17,12);//D1把别的给灭了
				 }
				if(!OLED_GetPoint(88,15)){
					 OLED_ReverseArea(88,15,17,12);//D2高亮
				 }
				if(OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3
				 }
				//D2
			}else{
				if(OLED_GetPoint(67,15)){//如果已经亮了就不管了 没亮就反转下
					 OLED_ReverseArea(67,15,17,12);//D1
				 }
				if(OLED_GetPoint(88,15)){//把别的给灭了
					 OLED_ReverseArea(88,15,17,12);//D2
				 }
				if(!OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3高亮
				 }
				//D3
			}
			}else if(menu ==1){ //切换舵机角度显示页面
		if (previousmenu!=1){
				OLED_ShowImage(0, 0, 128, 64, image_data2);
				OLED_ReverseArea(0,0,129,64);
			}
			previousmenu=menu;
			for (int i= 0; i < 10; i++) {//循环10次遍历舵机数据
				OLED_ClearArea(2+13*i,16,7,35);//清除原来舵机的角度条 最大高度 35
				OLED_ReverseArea(3+13*i,50-(int)((float)stat.servo[i]/180*33),5,(int)((float)stat.servo[i]/180*33));//绘制新的角度条 高度33
		}
	}
		OLED_Update();
}
void UART_UpdateInfo(void) {
		//待修改
}

void vNailongTask(void *pvParameters) //定时数据更新 20HZ
{
			TickType_t preTime;
			preTime = xTaskGetTickCount();//记录起始时间 时间起点
			uint8_t count =0;
			for (;;) {
				count++;
				CanOpen_UpdateInfo(); //更新电机数据
				if(count>=6){
					OLED_UpdateInfo(menu); // 执行OLED更新操作 降低OLED刷新对系统的负载
					count = 0;
				}
				vTaskDelayUntil(&preTime, 33); //时间终点
			}
}


void NailongTask_Init(void){
	const osThreadAttr_t NailongTask_attributes = {
    .name       = "Nailong",
    .priority   = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 8
	};
	NailongTaskHandle = osThreadNew(vNailongTask, NULL, &NailongTask_attributes);
}

extern osThreadId_t ServoTaskHandle;
extern osThreadId_t btnTaskHandle;

void EnterSafemode(uint8_t Errcode){ //安全模式	 Bug:有的时候无法显示错误代码
	CANopen_StopMotor();//电机解轴
	stat.Ready = false;
	//舵机解轴
	StopAllServo(); 
	//暂停其他任务
	vTaskDelete(NailongTaskHandle); //该线程参与对oled的刷新
	vTaskDelete(ServoTaskHandle);
	vTaskDelete(btnTaskHandle);
	vTaskDelay(100);
	taskENTER_CRITICAL();
	OLED_Init();
	OLED_Clear();
	OLED_ShowImage(41, 5, 45, 44, ERRORData); 
	//显示错误代码
	sprintf(buffer, "ERROR CODE:0x%02X", Errcode); 
	OLED_ShowString(20, 50, buffer, OLED_6X8);
	OLED_Update();
	taskEXIT_CRITICAL();
}

void QuitSafemode(void){ //退出安全模式
	SystemReset();
//	printf("QuitSafemode\r\n");
//	stat.Ready = true;
//	vTaskResume(NailongTaskHandle);//恢复任务
//	vTaskResume(ServoTaskHandle);
//	vTaskResume(btnTaskHandle);
//	OLED_Init();
//	previousmenu=1;
//	menu = 0;
//	if(stat.motorEN ){
//		//CANopen_SetSpeedMode();  //设置电机速度模式
//		CANopen_StartMotor();		//使能电机	
//	}
}
