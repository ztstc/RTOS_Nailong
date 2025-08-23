/*UARTDaemonTask.c*/
//串口DMA输出&队列和线程安全
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "UARTDaemonTask.h"
//#include "UARTProtocol.h"

extern UART_HandleTypeDef huart1;

osSemaphoreId_t xTxSemaphore;     // 发送完成信号量

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t data = (uint8_t)(huart->Instance->DR & 0xFF);
        osMessageQueuePut(xUartRxQueue, &data, 0, 0); 
        HAL_UART_Receive_IT(huart, &data, 1);
    }
}

// 修改后的守护任务
void vUartDaemonTask(void *pvParameters) {
    uart_msg_t msg;
    uint8_t rx_data;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	
    for (;;) {
        if (osMessageQueueGet(xUartTxQueue, &msg, NULL, osWaitForever) == osOK) {
            // 使用信号量等待发送完成
            HAL_UART_Transmit_DMA(&huart1, msg.data, msg.len);
            osSemaphoreAcquire(xTxSemaphore, osWaitForever);
            
            // 发送完成后释放内存
            vPortFree(msg.data);
        }
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 释放信号量通知任务完成
        osSemaphoreRelease(xTxSemaphore);
    }
}

osThreadId_t UartDaemonTaskHandle;
void vUartDaemonInit(void) {
    // 创建队列和信号量
    xUartTxQueue = osMessageQueueNew(UART_TX_QUEUE_LEN, sizeof(uart_msg_t), NULL);
    xUartRxQueue = osMessageQueueNew(UART_RX_QUEUE_LEN, sizeof(uint8_t), NULL);  // 初始化接收队列
    xTxSemaphore = osSemaphoreNew(1, 0, NULL);  // 二值信号量

    //UART_Protocol_Init();
    
    const osThreadAttr_t uartDaemonTask_attributes = {
        .name = "UARTDaemon",
        .stack_size =256 ,
        .priority = osPriorityHigh,
    };
    UartDaemonTaskHandle =osThreadNew(vUartDaemonTask, NULL, &uartDaemonTask_attributes);
}


void printf_daemon(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    uint8_t *buf = (uint8_t *)pvPortMalloc(UART_TX_MAX_LEN);
    if (!buf) {
        va_end(args);
        return;
    }

    int len = vsnprintf((char *)buf, UART_TX_MAX_LEN, fmt, args);
    va_end(args);

    if (len > 0) {
        uint16_t send_len = len < UART_TX_MAX_LEN ? len : UART_TX_MAX_LEN - 1;
        
        uart_msg_t msg = {
            .data = buf,
            .len  = send_len
        };

        if (osMessageQueuePut(xUartTxQueue, &msg, 0, 0) != osOK) {
            vPortFree(buf);
        }
    } else {
        vPortFree(buf);
    }
}
