#ifndef UART_DAEMON_TASK_H
#define UART_DAEMON_TASK_H

#include "cmsis_os.h"

#define UART_TX_QUEUE_LEN  16
#define UART_TX_MAX_LEN    256
#define UART_RX_QUEUE_LEN  64  // 新增接收队列长度

typedef struct {
    uint8_t *data;
    uint16_t len;
} uart_msg_t;

extern osMessageQueueId_t xUartTxQueue;  // 移除static
extern osMessageQueueId_t xUartRxQueue;  // 新增接收队列声明
extern osThreadId_t UartDaemonTaskHandle;

void vUartDaemonTask(void *pvParameters);
void vUartDaemonInit(void);
void printf_daemon(const char *fmt, ...);

#endif
