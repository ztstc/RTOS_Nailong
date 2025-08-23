#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <string.h>
#include "Nailong.h"
#include "CanOpenSDO.h"

////CPU和RAM占用计算
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

    /* 找到空闲任务 */
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

    /* 总 CPU 占用 = 100 - 空闲任务占比 */
    return 100.0f - (ulIdleTime * 100.0f) / ulTotalRunTime;
}

/* 任务函数 */
void vCpuStatTask(void *argument)
{
	/* CPU 占用百分比，0~100 */
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


/*====任务状态打印====*/

/*https://blog.csdn.net/yqqsyjnh/article/details/120661396*/
uint8_t InfoBuffer[1000];

void vCPU_RunTime(void *argument)  //仅用作调试 超过59分钟可能会出现异常
{
		TickType_t preTime;
		preTime = xTaskGetTickCount();//记录起始时间 时间起点
	//printf("vCPU_RunTime Start!\r\n");
	while(1)
		{
		taskENTER_CRITICAL();
		 vTaskList((char *)&InfoBuffer);
		 printf("=================================================\r\n");
		 printf("任务名        任务状态 优先级  剩余栈  任务号 \r\n");
		 printf("=================================================\r\n");
		 printf("%s\r\n", InfoBuffer);
		 printf("  B：阻塞  R：就绪  D：删除  S：暂停  X：运行 \r\n");
		 printf("=================================================\r\n");
		 printf("=================================================\r\n");
		 printf("任务名         运行计数        CPU使用率 \r\n");
		 printf("=================================================\r\n");
		 vTaskGetRunTimeStats((char *)&InfoBuffer); 
		 printf("%s\r\n",InfoBuffer);
		 printf("=================================================\r\n");
		 printf("=================================================\r\n\n\n");
		 taskEXIT_CRITICAL();
		 vTaskDelayUntil(&preTime, 1300); //时间终点
    }	
}

osThreadId_t TaskStatTaskHandle;

void TaskStatTask_Init(void){
		
		const osThreadAttr_t taskStatTask_attributes = {
			.name       = "taskStat",
			.priority   = (osPriority_t) osPriorityAboveNormal, /* 优先级高于普通任务 */
			.stack_size = 512
	};
		TaskStatTaskHandle = osThreadNew(vCPU_RunTime, NULL, &taskStatTask_attributes);
}



/*====CPU占用情况更新 利用IDLE钩子函数实现====*/

osThreadId_t cpuStatTaskHandle;

void CpuStatTask_Init(void){
		
		const osThreadAttr_t cpuStatTask_attributes = {
			.name       = "cpuStat",
			.priority   = (osPriority_t) osPriorityAboveNormal, /* 优先级需要高于普通任务 */
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

/*====重启单片机====*/
void SystemReset(void){
	__disable_irq();
	HAL_NVIC_SystemReset();
}

