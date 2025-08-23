/* CPU_Stat.h */
#include "cmsis_os.h"

/* 供外部使用的声明 */
extern osThreadId_t cpuStatTaskHandle;
extern const osThreadAttr_t cpuStatTask_attributes;
void CpuStatTask_Init(void);
void vCpuStatTask(void *argument);
void idletimeCounter(void);
void TaskStatTask_Init(void);
void SystemReset(void);
