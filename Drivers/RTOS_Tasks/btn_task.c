#include "btn_task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Nailong.h"
#include "CanOpenSDO.h"

/* -------------------- �û��ɸĲ��� -------------------- */
#define SHORT_PRESS_MS   50     // �̰���Сʱ�䣨ȥ����
#define LONG_PRESS_MS   500    // �����ж���ֵ
#define SAMPLING_MS       10    // ÿ 10 ms ����һ��
/*-------------------------------------------------------*/

/* �¼���־λ */
#define BTN_EVT_SHORT  (1 << 0)
#define BTN_EVT_LONG   (1 << 1)

/* ȫ�־�� */
osThreadId_t btnTaskHandle;
static const osThreadAttr_t btnTaskAttr = {
    .name = "btnTask",
    .stack_size = 128*4,
    .priority = osPriorityNormal,
};

extern uint8_t menu;

/* �ڲ����� ------------------------------------------------*/
static uint8_t btn_read(void)
{
    /* ����Ϊ�͵�ƽ������ 1���ɿ�Ϊ�ߵ�ƽ������ 0 */
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1 : 0;
}

static void btn_task(void *argument)
{
    uint32_t pressTime = 0;
    uint8_t  last = 0, curr = 0;
    uint8_t longPressTriggered = 0; // �����Ѵ�����־

    for (;;)
    {
        curr = btn_read();
        
        /* ����״̬�仯��� */
        if (last != curr) 
        {
            /* �����¼����½��أ� */
            if (curr && !last) 
            {
                pressTime = 0;
                longPressTriggered = 0; // ���ó�����־
            }
            /* �ɿ��¼��������أ� */
            else if (!curr && last) 
            {
                /* ����δ��������ʱ�Ŵ���̰� */
                if (!longPressTriggered && pressTime >= SHORT_PRESS_MS) 
                {
                    //printf("Short press (%d ms)\r\n", pressTime);
                    menu++; // �л��˵�
                    menu %= 2; // ���Ʋ˵���Χ
                }
            }
        }

        /* �����������´��� */
        if (curr) 
        {
            pressTime += SAMPLING_MS;
            
            /* �ﵽ������ֵ��δ������ */
            if (!longPressTriggered && pressTime >= LONG_PRESS_MS) 
            {
                longPressTriggered = 1; // ����Ѵ���
                //printf("Long press detected (%d ms)\r\n", pressTime);
							if(stat.motorEN){
								CANopen_StopMotor();
							}else{
								CANopen_StartMotor();
							}
                //printf("MotorEN %d\r\n", stat.motorEN);
            }
        }

        last = curr;
        osDelay(SAMPLING_MS);
    }
}

/* ����ӿڣ��������� ------------------------------------------------*/
void btn_task_create(void)
{
    btnTaskHandle = osThreadNew(btn_task, NULL, &btnTaskAttr);
}
