#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <string.h>
#include "Nailong.h"
#include "CanOpenSDO.h"

////CPU��RAMռ�ü���
static volatile uint32_t ulIdleCycleCount = 0UL;
//static uint32_t ulIdleCycleLast   = 0UL; 
//static uint32_t ulIdleCycleDiff   = 0UL; 

struct SYS
{
   int   CPU;
   int   RAM;
} sys= {0,0};

void idletimeCounter(void){
		ulIdleCycleCount++;
}

#include "task.h"

float GetTotalCpuUsagePercent(void)
{
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize;
    uint32_t ulTotalRunTime;

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    if (pxTaskStatusArray == NULL)
        return -1.0f;

    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
    if (ulTotalRunTime == 0)
    {
        vPortFree(pxTaskStatusArray);
        return 0.0f;
    }

    /* �ҵ��������� */
    uint32_t ulIdleTime = 0;
    for (UBaseType_t i = 0; i < uxArraySize; i++)
    {
        if (strstr(pxTaskStatusArray[i].pcTaskName, "IDLE") != NULL)
        {
            ulIdleTime = pxTaskStatusArray[i].ulRunTimeCounter;
            break;
        }
    }

    vPortFree(pxTaskStatusArray);

    /* �� CPU ռ�� = 100 - ��������ռ�� */
    return 100.0f - (ulIdleTime * 100.0f) / ulTotalRunTime;
}

/* ������ */
void vCpuStatTask(void *argument)
{
	/* CPU ռ�ðٷֱȣ�0~100 */
	size_t total = configTOTAL_HEAP_SIZE;
    for (;;)
    {
        float cpu = GetTotalCpuUsagePercent();
				//printf("CPU:%.1f%%\r\n", cpu);
				vTaskDelay(pdMS_TO_TICKS(1000));
				stat.CPU=(int)(cpu);
				size_t free = xPortGetFreeHeapSize();
				stat.RAM = (free * 100) / total;
    }
}


/*====����״̬��ӡ====*/

/*https://blog.csdn.net/yqqsyjnh/article/details/120661396*/
uint8_t InfoBuffer[1000];

void vCPU_RunTime(void *argument)  //���������� ����59���ӿ��ܻ�����쳣
{
		TickType_t preTime;
		preTime = xTaskGetTickCount();//��¼��ʼʱ�� ʱ�����
	//printf("vCPU_RunTime Start!\r\n");
	while(1)
		{
		taskENTER_CRITICAL();
		 vTaskList((char *)&InfoBuffer);
		 printf("=================================================\r\n");
		 printf("������        ����״̬ ���ȼ�  ʣ��ջ  ����� \r\n");
		 printf("=================================================\r\n");
		 printf("%s\r\n", InfoBuffer);
		 printf("  B������  R������  D��ɾ��  S����ͣ  X������ \r\n");
		 printf("=================================================\r\n");
		 printf("=================================================\r\n");
		 printf("������         ���м���        CPUʹ���� \r\n");
		 printf("=================================================\r\n");
		 vTaskGetRunTimeStats((char *)&InfoBuffer); 
		 printf("%s\r\n",InfoBuffer);
		 printf("=================================================\r\n");
		 printf("=================================================\r\n\n\n");
		 taskEXIT_CRITICAL();
		 vTaskDelayUntil(&preTime, 1300); //ʱ���յ�
    }	
}

osThreadId_t TaskStatTaskHandle;

void TaskStatTask_Init(void){
		
		const osThreadAttr_t taskStatTask_attributes = {
			.name       = "taskStat",
			.priority   = (osPriority_t) osPriorityAboveNormal, /* ���ȼ�������ͨ���� */
			.stack_size = 512
	};
		TaskStatTaskHandle = osThreadNew(vCPU_RunTime, NULL, &taskStatTask_attributes);
}



/*====CPUռ��������� ����IDLE���Ӻ���ʵ��====*/

osThreadId_t cpuStatTaskHandle;

void CpuStatTask_Init(void){
		
		const osThreadAttr_t cpuStatTask_attributes = {
			.name       = "cpuStat",
			.priority   = (osPriority_t) osPriorityAboveNormal, /* ���ȼ���Ҫ������ͨ���� */
			.stack_size = 512
	};
		cpuStatTaskHandle = osThreadNew(vCpuStatTask, NULL, &cpuStatTask_attributes);
}

void HeapUsage(void)
{
    size_t total = configTOTAL_HEAP_SIZE;
    size_t free = xPortGetFreeHeapSize();
    sys.RAM = (free * 100) / total;
}

/*====������Ƭ��====*/
void SystemReset(void){
	__disable_irq();
	HAL_NVIC_SystemReset();
}

