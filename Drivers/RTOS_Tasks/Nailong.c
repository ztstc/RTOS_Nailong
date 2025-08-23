/*Nailong.c*/
/*����С���������ƿ�*/
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

// ���岢��ʼ��ȫ�ֱ���
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
char buffer[64]; 								//ת����ʽ�õĻ�����
char txBuffer[128]; 					// ���ڷ��͸���λ��������

void ButtonLED(bool stat){ //����Reset��ť��ָʾ��
	 if(stat){
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	 }else{
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 }
}

// ���ȫ���ж��Ƿ����� =====����=====
void Print_Global_IRQ_Status(void) {
    // ʹ���ڲ�������ȡPRIMASK
    uint32_t primask = __get_PRIMASK();
    
    if (primask & 1) {
        printf("[IRQ] Global interrupts DISABLED\n");
    } else {
        printf("[IRQ] Global interrupts ENABLED\n");
    }
}

void Nailong_Init(void){
	//�ڵ���������ǰ������ʹ��vTaskDelay��osDelay
	ButtonLED(true);
	HAL_Delay(400);
	ButtonLED(false);
	HAL_Delay(400);
	ButtonLED(true);
	/* OLED��ʼ�� */
	OLED_Init();//�˴���ʼ HAL_Delay �ᵼ������
	OLED_ShowImage(0, 0, 128, 64, image_data3); //��ʾ����ͼ��
	OLED_Update(); //������Ļ���棨�����²����иı䣩
	/* CAN��ʼ�� */
	CANopen_Init();
	HAL_CAN_Start(&hcan);
	CANopen_SetSpeedResolution(0xA);//�ٶȷֱ���0.1 ���ñ��������õ� ����Ҫ��Ū�� �����������Ȼ�������ں����ڲ������ʻ��� 
	//CANopen_SaveToEEPROM();//���ñ��������õ� ����Ҫ��Ū��
	CANopen_SetMotorAccelDecel(500, 500);//�Ӽ���ʱ��1500ms
	CANopen_SetSpeedMode();  //���õ���ٶ�ģʽ
	CANopen_StartMotor();		//ʹ�ܵ��	
//	CANopen_SetTargetSpeed(2, 2); //����Ŀ���ٶ�
//	HAL_Delay(100);
	CANopen_SetTargetSpeed(0, 0);
	HAL_Delay(100);
	CANopen_SetSpeedLimit(50);	//�趨����ٶ�

	/*���߶����ʼ��*/
	ServoBus_Init(&huart2); //��ʼ�����߶��
	ServoBus_StartReadTask();
	
	// �������߶��
	//uint8_t servo_count = DiscoverServos(stat.onlineservoid, 10);
	//printf("Found %d online servos\n", servo_count);//����ʹ�ø�ָ��
	//Ϊ���ߵĶ�����ͽ���ָ��
//	for(int i = 0;i<10;i++){
//		if (stat.onlineservoid[i] != 0xFF) {
//			ServoBus_SendCommand(stat.onlineservoid[i],SERVO_CMD_RELEASE_TORQUE,0,0);
//    } 
//	}
	
	for(int pixel=0;pixel<63;pixel+=5){ //����С����
		OLED_ReverseArea(0,pixel,129,5); //��5�з�ɫ �и�����������
		OLED_Update();
	}
	
	OLED_ShowImage(0, 0, 128, 64, image_data1); //�л�Ϊ���˵�
	OLED_ShowString(0, 3, "Pre", OLED_6X8); //��ʾ��ʼ����
	OLED_ShowString(38, 3, "0.0V", OLED_6X8);
	OLED_ShowString(72, 3, "0.0A", OLED_6X8);
	OLED_ShowString(5, 33, "0", OLED_6X8);
	OLED_ShowString(5, 51, "0", OLED_6X8);
	OLED_ShowString(75, 33, "0", OLED_6X8);
	OLED_ShowString(75, 51, "0", OLED_6X8);
	OLED_DrawRectangle(106, 3, 18, 6, 1); // ���Ƶ�����
	OLED_Update();
}

void OLED_UpdateInfo(uint8_t menu){//oled��������
		if(menu == 0){
			OLED_ShowImage(0, 0, 128, 64, image_data1);
			if(stat.Ready == true && stat.motorEN == true){
				OLED_ClearArea(0, 3, 38, 8);
				OLED_ShowString(0, 3, "Ready", OLED_6X8);
			}else{
				OLED_ClearArea(0, 3, 38, 8); 
				OLED_ShowString(0, 3, "Stop", OLED_6X8); //��ͣ��ť������ �� ���û��ʹ��
			}
			previousmenu=menu;
			
			snprintf(buffer, sizeof(buffer), "%.1fV", stat.batteryVoltage);//��ѹ
			OLED_ClearArea(38,3,30,8);//��յ�ѹ����
			OLED_ClearArea(106,3,18,6);//��յ����
				/*����18 25.5-26.5V
			����15 24.8-25.5V
			����12 24.0-24.8V
			����9  23.2-24.0V
			����6  22.4-23.2V
			����3  <22.4V (���)
			 */
//						if(stat.batteryVoltage>=25.5){
//				OLED_DrawRectangle(106,3,18,6,1);//�����
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
					int length = 1 + (int)(ratio * 17 + 0.5);   // ��������ȡ������Χ1~18
					OLED_DrawRectangle(106, 3, length, 6, 1);
			}
			OLED_ShowString(38, 3, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1fA", stat.motorCurrent);//�ܵ���
			OLED_ClearArea(72,3,30,8);
			OLED_ShowString(72, 3, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.req_speedL);//����Ŀ��ת��
			OLED_ClearArea(5,33,24,8);
			OLED_ShowString(5, 33, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.req_speedR);//�ҵ��Ŀ��ת��
			OLED_ClearArea(5,51,24,8);
			OLED_ShowString(5, 51, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.speedL);//����ʵ��ת��
			OLED_ClearArea(75,33,24,8);
			OLED_ShowString(75, 33, buffer, OLED_6X8);
			snprintf(buffer, sizeof(buffer), "%.1f", stat.speedR);//�ҵ��ʵ��ת��
			OLED_ClearArea(75,51,24,8);
			OLED_ShowString(75, 51, buffer, OLED_6X8);
			if(stat.req_speedL==0 && stat.req_speedR==0 && stat.motorEN==true){ //P������
				 if(!OLED_GetPoint(37,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(37,15,13,12);//P����(��ת��ʾ��
				 }
				 if(OLED_GetPoint(52,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(52,15,13,12);//N�䰵(��ת��ʾ��
				 }
			}else if(stat.req_speedL==0 && stat.req_speedR==0 && stat.motorEN==false){
					if(OLED_GetPoint(37,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(37,15,13,12);//P�䰵(��ת��ʾ��
				 }
					if(!OLED_GetPoint(52,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(52,15,13,12);//N����(��ת��ʾ��
				 }
			 }else{
				 if(OLED_GetPoint(37,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(37,15,13,12);//P�䰵(��ת��ʾ��
				 }
					if(OLED_GetPoint(52,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(52,15,13,12);//N�䰵(��ת��ʾ��
				 }
			 }
			if(stat.speedLimit<=25){
				if(!OLED_GetPoint(67,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(67,15,17,12);//D1����
				 }
				if(OLED_GetPoint(88,15)){//�ѱ�ĸ�����
					 OLED_ReverseArea(88,15,17,12);//D2
				 }
				if(OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3
				 }
				//D1��
			}else if(stat.speedLimit<=50){
				if(OLED_GetPoint(67,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(67,15,17,12);//D1�ѱ�ĸ�����
				 }
				if(!OLED_GetPoint(88,15)){
					 OLED_ReverseArea(88,15,17,12);//D2����
				 }
				if(OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3
				 }
				//D2
			}else{
				if(OLED_GetPoint(67,15)){//����Ѿ����˾Ͳ����� û���ͷ�ת��
					 OLED_ReverseArea(67,15,17,12);//D1
				 }
				if(OLED_GetPoint(88,15)){//�ѱ�ĸ�����
					 OLED_ReverseArea(88,15,17,12);//D2
				 }
				if(!OLED_GetPoint(110,15)){
					 OLED_ReverseArea(110,15,17,12);//D3����
				 }
				//D3
			}
			}else if(menu ==1){ //�л�����Ƕ���ʾҳ��
		if (previousmenu!=1){
				OLED_ShowImage(0, 0, 128, 64, image_data2);
				OLED_ReverseArea(0,0,129,64);
			}
			previousmenu=menu;
			for (int i= 0; i < 10; i++) {//ѭ��10�α����������
				OLED_ClearArea(2+13*i,16,7,35);//���ԭ������ĽǶ��� ���߶� 35
				OLED_ReverseArea(3+13*i,50-(int)((float)stat.servo[i]/180*33),5,(int)((float)stat.servo[i]/180*33));//�����µĽǶ��� �߶�33
		}
	}
		OLED_Update();
}
void UART_UpdateInfo(void) {
		//���޸�
}

void vNailongTask(void *pvParameters) //��ʱ���ݸ��� 20HZ
{
			TickType_t preTime;
			preTime = xTaskGetTickCount();//��¼��ʼʱ�� ʱ�����
			uint8_t count =0;
			for (;;) {
				count++;
				CanOpen_UpdateInfo(); //���µ������
				if(count>=6){
					OLED_UpdateInfo(menu); // ִ��OLED���²��� ����OLEDˢ�¶�ϵͳ�ĸ���
					count = 0;
				}
				vTaskDelayUntil(&preTime, 33); //ʱ���յ�
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

void EnterSafemode(uint8_t Errcode){ //��ȫģʽ	 Bug:�е�ʱ���޷���ʾ�������
	CANopen_StopMotor();//�������
	stat.Ready = false;
	//�������
	StopAllServo(); 
	//��ͣ��������
	vTaskDelete(NailongTaskHandle); //���̲߳����oled��ˢ��
	vTaskDelete(ServoTaskHandle);
	vTaskDelete(btnTaskHandle);
	vTaskDelay(100);
	taskENTER_CRITICAL();
	OLED_Init();
	OLED_Clear();
	OLED_ShowImage(41, 5, 45, 44, ERRORData); 
	//��ʾ�������
	sprintf(buffer, "ERROR CODE:0x%02X", Errcode); 
	OLED_ShowString(20, 50, buffer, OLED_6X8);
	OLED_Update();
	taskEXIT_CRITICAL();
}

void QuitSafemode(void){ //�˳���ȫģʽ
	SystemReset();
//	printf("QuitSafemode\r\n");
//	stat.Ready = true;
//	vTaskResume(NailongTaskHandle);//�ָ�����
//	vTaskResume(ServoTaskHandle);
//	vTaskResume(btnTaskHandle);
//	OLED_Init();
//	previousmenu=1;
//	menu = 0;
//	if(stat.motorEN ){
//		//CANopen_SetSpeedMode();  //���õ���ٶ�ģʽ
//		CANopen_StartMotor();		//ʹ�ܵ��	
//	}
}
