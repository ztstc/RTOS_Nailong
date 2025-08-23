/*UARTDaemonTask.c*/
//����DMA���&���к��̰߳�ȫ
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

osSemaphoreId_t xTxSemaphore;     // ��������ź���

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t data = (uint8_t)(huart->Instance->DR & 0xFF);
        osMessageQueuePut(xUartRxQueue, &data, 0, 0); 
        HAL_UART_Receive_IT(huart, &data, 1);
    }
}

// �޸ĺ���ػ�����
void vUartDaemonTask(void *pvParameters) {
    uart_msg_t msg;
    uint8_t rx_data;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	
    for (;;) {
        if (osMessageQueueGet(xUartTxQueue, &msg, NULL, osWaitForever) == osOK) {
            // ʹ���ź����ȴ��������
            HAL_UART_Transmit_DMA(&huart1, msg.data, msg.len);
            osSemaphoreAcquire(xTxSemaphore, osWaitForever);
            
            // ������ɺ��ͷ��ڴ�
            vPortFree(msg.data);
        }
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // �ͷ��ź���֪ͨ�������
        osSemaphoreRelease(xTxSemaphore);
    }
}

osThreadId_t UartDaemonTaskHandle;
void vUartDaemonInit(void) {
    // �������к��ź���
    xUartTxQueue = osMessageQueueNew(UART_TX_QUEUE_LEN, sizeof(uart_msg_t), NULL);
    xUartRxQueue = osMessageQueueNew(UART_RX_QUEUE_LEN, sizeof(uint8_t), NULL);  // ��ʼ�����ն���
    xTxSemaphore = osSemaphoreNew(1, 0, NULL);  // ��ֵ�ź���

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
