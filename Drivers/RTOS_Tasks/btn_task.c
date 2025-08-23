#include "btn_task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Nailong.h"
#include "CanOpenSDO.h"

/* -------------------- 用户可改参数 -------------------- */
#define SHORT_PRESS_MS   50     // 短按最小时间（去抖后）
#define LONG_PRESS_MS   500    // 长按判定阈值
#define SAMPLING_MS       10    // 每 10 ms 采样一次
/*-------------------------------------------------------*/

/* 事件标志位 */
#define BTN_EVT_SHORT  (1 << 0)
#define BTN_EVT_LONG   (1 << 1)

/* 全局句柄 */
osThreadId_t btnTaskHandle;
static const osThreadAttr_t btnTaskAttr = {
    .name = "btnTask",
    .stack_size = 128*4,
    .priority = osPriorityNormal,
};

extern uint8_t menu;

/* 内部函数 ------------------------------------------------*/
static uint8_t btn_read(void)
{
    /* 按下为低电平，返回 1；松开为高电平，返回 0 */
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1 : 0;
}

static void btn_task(void *argument)
{
    uint32_t pressTime = 0;
    uint8_t  last = 0, curr = 0;
    uint8_t longPressTriggered = 0; // 长按已触发标志

    for (;;)
    {
        curr = btn_read();
        
        /* 按键状态变化检测 */
        if (last != curr) 
        {
            /* 按下事件（下降沿） */
            if (curr && !last) 
            {
                pressTime = 0;
                longPressTriggered = 0; // 重置长按标志
            }
            /* 松开事件（上升沿） */
            else if (!curr && last) 
            {
                /* 仅当未触发长按时才处理短按 */
                if (!longPressTriggered && pressTime >= SHORT_PRESS_MS) 
                {
                    //printf("Short press (%d ms)\r\n", pressTime);
                    menu++; // 切换菜单
                    menu %= 2; // 限制菜单范围
                }
            }
        }

        /* 按键持续按下处理 */
        if (curr) 
        {
            pressTime += SAMPLING_MS;
            
            /* 达到长按阈值且未触发过 */
            if (!longPressTriggered && pressTime >= LONG_PRESS_MS) 
            {
                longPressTriggered = 1; // 标记已触发
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

/* 对外接口：创建任务 ------------------------------------------------*/
void btn_task_create(void)
{
    btnTaskHandle = osThreadNew(btn_task, NULL, &btnTaskAttr);
}
