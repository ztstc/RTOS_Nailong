#include "Servo.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "Nailong.h"

// 全局变量定义
UART_HandleTypeDef* g_huart = NULL;
float servo_angles[10] = {0};  // 10个舵机的角度值
uint8_t online_servos[10] = {0}; // 在线舵机状态 (0:离线, 1:在线)
const uint8_t MAX_SERVO_ID = 9; // 最大舵机ID

// 初始化舵机库
void ServoBus_Init(UART_HandleTypeDef* huart) {
    g_huart = huart;
    // 初始全部标记为离线
    memset(online_servos, 0, sizeof(online_servos));
}

// 清空UART接收缓冲区
static void UART_FlushRxBuffer(void) {
    uint8_t dummy;
    // 读取所有接收到的数据直到为空
    while (HAL_UART_Receive(g_huart, &dummy, 1, 0) == HAL_OK) {
        // 丢弃数据
    }
}

// 发送控制指令
void ServoBus_SendCommand(uint8_t id, ServoCommand cmd, uint16_t param1, uint16_t param2) {
    if(id > MAX_SERVO_ID) return; // 只处理0-9的舵机ID
    
    char command[32];
    memset(command, 0, sizeof(command));

    switch (cmd) {
        case SERVO_CMD_CONTROL:
            sprintf(command, "#%03dP%04dT%04d!", id, param1, param2);
            break;
        case SERVO_CMD_READ_VERSION:
            sprintf(command, "#%03dPVER!", id);
            break;
        case SERVO_CMD_READ_ID:
            sprintf(command, "#%03dPID!", id);
            break;
        case SERVO_CMD_SET_ID:
            sprintf(command, "#%03dPID%03d!", id, param1);
            break;
        case SERVO_CMD_RELEASE_TORQUE:
            sprintf(command, "#%03dPULK!", id);
            break;
        case SERVO_CMD_RESTORE_TORQUE:
            sprintf(command, "#%03dPULR!", id);
            break;
        case SERVO_CMD_READ_MODE:
            sprintf(command, "#%03dPMOD!", id);
            break;
        case SERVO_CMD_SET_MODE:
            sprintf(command, "#%03dPMOD%d!", id, param1);
            break;
        case SERVO_CMD_READ_POSITION:
            sprintf(command, "#%03dPRAD!", id);
            break;
        case SERVO_CMD_PAUSE:
            sprintf(command, "#%03dPDPT!", id);
            break;
        case SERVO_CMD_CONTINUE:
            sprintf(command, "#%03dPDCT!", id);
            break;
        case SERVO_CMD_STOP:
            sprintf(command, "#%03dPDST!", id);
            break;
        case SERVO_CMD_SET_BAUDRATE:
            sprintf(command, "#%03dPBD%d!", id, param1);
            break;
        case SERVO_CMD_CALIBRATE:
            sprintf(command, "#%03dPSCK!", id);
            break;
        case SERVO_CMD_SET_START_POSITION:
            sprintf(command, "#%03dPCSD!", id);
            break;
        case SERVO_CMD_REMOVE_START_POSITION:
            sprintf(command, "#%03dPCSM!", id);
            break;
        case SERVO_CMD_RESTORE_START_POSITION:
            sprintf(command, "#%03dPCSR!", id);
            break;
        case SERVO_CMD_SET_MIN_VALUE:
            sprintf(command, "#%03dPSMI!", id);
            break;
        case SERVO_CMD_SET_MAX_VALUE:
            sprintf(command, "#%03dPSMX!", id);
            break;
        case SERVO_CMD_RESET_TO_DEFAULT:
            sprintf(command, "#%03dPCLE!", id);
            break;
        case SERVO_CMD_READ_VOLTAGE_TEMPERATURE:
            sprintf(command, "#%03dPRTV!", id);
            break;
        default:
            break;
    }

    // 直接发送命令（无互斥保护）
    HAL_UART_Transmit(g_huart, (uint8_t*)command, strlen(command), 10);
}

// 解析舵机返回消息
float ServoParseMessage(const char* response) {
    if(response == NULL || strlen(response) < 10) 
        return -1.0f;
    
    // 基本格式验证 (期望格式: "#001P1500!")
    if(response[0] != '#' || response[4] != 'P' || response[9] != '!') 
        return -1.0f;
    
    // 提取PWM值 (4位数字)
    char pwm_str[5] = {0};
    strncpy(pwm_str, &response[5], 4);
    
    // 转换为数字
    char* endptr;
    long pwm_value = strtol(pwm_str, &endptr, 10);
    if(endptr == pwm_str || pwm_value < 500 || pwm_value > 2500) 
        return -1.0f;
    
    // 转换PWM值为角度 (500-2500 → 0-270°)
    return ((float)(pwm_value - 500) / 2000.0f) * 180.0f;
}

// 舵机角度读取任务函数
// 舵机角度读取任务函数
void vServo_ReadTask(void *pvParameters) {
    uint8_t current_id = 0;
    uint8_t cycle = 0;
    uint8_t buffer[11]; // 10字节数据 + 1字节终止符
    
    // 初始扫描在线舵机
    //printf("Found %d online servos\n", DiscoverServos());
    DiscoverServos();
    for(;;) {
        // 只处理在线的舵机
        if(online_servos[current_id]) {
            // 清空接收缓冲区
            UART_FlushRxBuffer();
            
            // 发送读取角度指令
						//printf("Reading: %d \n",current_id);
            ServoBus_SendCommand(current_id, SERVO_CMD_READ_POSITION, 0, 0);
            
            // 接收响应 (期望10字节: "#001P1500!")
            HAL_StatusTypeDef status = HAL_UART_Receive(g_huart, buffer, 10, 100);
            
            if(status == HAL_OK) {
                buffer[10] = '\0'; // 添加字符串终止符
                //printf("RX: %.*s\n", sizeof(buffer), (char*)buffer);
                // 解析响应
                float result = ServoParseMessage((char*)buffer);
                if(result >= 0) {
                    servo_angles[current_id] = result;
									stat.servo[current_id] = servo_angles[current_id];
									osDelay(10);
                }
            } 
            

        }
        
        // 切换到下一个舵机ID (0-9)
        current_id = (current_id + 1) % 10;
        
        // 每20次扫描循环扫描一次在线状态
        if(current_id == 0) {
            cycle++;
            if(cycle >= 20) {
                //printf("Scanning all servos...\n");
               // printf("Found %d online servos\n", DiscoverServos());
                cycle = 0;
            }
        }
        
        osDelay(50); // 50ms延迟
    }
}

// 启动舵机角度读取任务
osThreadId_t ServoTaskHandle;

void ServoBus_StartReadTask(void) {
    const osThreadAttr_t ServoTask_attributes = {
        .name       = "Servo",
        .priority   = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
    };
    ServoTaskHandle = osThreadNew(vServo_ReadTask, NULL, &ServoTask_attributes);
}

// 设置舵机角度 (0-180°)
void SetServoAngle(uint8_t id, uint16_t angle, uint16_t time) {
    if(id > MAX_SERVO_ID) return; // 只处理0-9的舵机ID
    
    // 角度范围限制
    if(angle > 180) angle = 180;
    
    // 转换为PWM (500-2500)
    uint16_t pwm = 500 + (angle * 2000 / 180);
    
    // 发送控制命令
    ServoBus_SendCommand(id, SERVO_CMD_CONTROL, pwm, time);
}

// 检查舵机是否在线 (非阻塞式)
uint8_t IsServoOnline(uint8_t id) {
    if(id > MAX_SERVO_ID) return 0;
    return online_servos[id];
}

// 发现在线舵机  (仅扫描0-9)(通过读取角度方式)
uint8_t DiscoverServos(void) {
    uint8_t count = 0;
    if(g_huart == NULL) return 0;
    
    // 只扫描0-9 ID
    for(uint8_t id = 0; id <= MAX_SERVO_ID; id++) {
        // 清空接收缓冲区
        UART_FlushRxBuffer();
        
        // 发送读取角度指令
        ServoBus_SendCommand(id, SERVO_CMD_READ_POSITION, 0, 0);
        // 接收响应 (带超时)
        uint8_t buffer[11] = {0};
        HAL_StatusTypeDef status = HAL_UART_Receive(g_huart, buffer, 10, 50);
        
        // 验证响应
        online_servos[id] = 0; // 默认离线
        
        if(status == HAL_OK) {
            buffer[10] = '\0'; // 添加字符串终止符
            
            // 解析角度响应
            float result = ServoParseMessage((char*)buffer);
            if(result >= 0) {
                // 解析成功，标记为在线并更新角度值
                online_servos[id] = 1;
								stat.onlineservoid[id] = 1;
                servo_angles[id] = result;
                count++;
							osDelay(50);
            }else{
							online_servos[id] = 0;
							stat.onlineservoid[id] = 0;
						}
        }
        
        // 短延迟防止总线拥塞
        osDelay(5);
    }
    return count;
}

void StopAllServo(void) {
    for(uint8_t id = 0; id <= MAX_SERVO_ID; id++) {
        if(online_servos[id]) {
            ServoBus_SendCommand(id, SERVO_CMD_RELEASE_TORQUE, 0, 0);
        }
    }
}
