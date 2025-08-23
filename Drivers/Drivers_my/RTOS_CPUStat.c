#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* user code 2 -------------------------------------------------------------*/
osThreadId_t cpuStatTaskHandle;
const osThreadAttr_t cpuStatTask_attributes = {
    .name       = "cpuStat",
    .priority   = (osPriority_t) osPriorityAboveNormal, /* 优先级高于普通任务 */
    .stack_size = 128
};

/* 任务函数 ---------------------------------------------------------------*/
void vCpuStatTask(void *argument)
{
    const uint32_t ulTotalPossible = 1000UL;   /* 假设1 s内空闲任务最多跑1000次 */
    uint32_t ulCpuLoad;                        /* CPU 占用百分比，0~100 */

    for (;;)
    {
        ulIdleCycleLast = ulIdleCycleCount;    /* 保存当前值 */
        osDelay(1000);                         /* 延时 1 s */

        /* 计算 1 s 内空闲任务实际跑了多少次 */
        ulIdleCycleDiff = ulIdleCycleCount - ulIdleCycleLast;

        /* 计算利用率：100 - 空闲百分比 */
        if (ulIdleCycleDiff >= ulTotalPossible)
            ulCpuLoad = 0;                     /* 防止溢出 */
        else
            ulCpuLoad = 100UL -
                        (ulIdleCycleDiff * 100UL) / ulTotalPossible;

        /* 串口打印（需重定向 printf） */
        printf("CPU Load: %lu %%\r\n", ulCpuLoad);
    }
}