#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* user code 2 -------------------------------------------------------------*/
osThreadId_t cpuStatTaskHandle;
const osThreadAttr_t cpuStatTask_attributes = {
    .name       = "cpuStat",
    .priority   = (osPriority_t) osPriorityAboveNormal, /* ���ȼ�������ͨ���� */
    .stack_size = 128
};

/* ������ ---------------------------------------------------------------*/
void vCpuStatTask(void *argument)
{
    const uint32_t ulTotalPossible = 1000UL;   /* ����1 s�ڿ������������1000�� */
    uint32_t ulCpuLoad;                        /* CPU ռ�ðٷֱȣ�0~100 */

    for (;;)
    {
        ulIdleCycleLast = ulIdleCycleCount;    /* ���浱ǰֵ */
        osDelay(1000);                         /* ��ʱ 1 s */

        /* ���� 1 s �ڿ�������ʵ�����˶��ٴ� */
        ulIdleCycleDiff = ulIdleCycleCount - ulIdleCycleLast;

        /* ���������ʣ�100 - ���аٷֱ� */
        if (ulIdleCycleDiff >= ulTotalPossible)
            ulCpuLoad = 0;                     /* ��ֹ��� */
        else
            ulCpuLoad = 100UL -
                        (ulIdleCycleDiff * 100UL) / ulTotalPossible;

        /* ���ڴ�ӡ�����ض��� printf�� */
        printf("CPU Load: %lu %%\r\n", ulCpuLoad);
    }
}