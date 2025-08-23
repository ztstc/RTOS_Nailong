#include "CanOpenSDO.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Nailong.h"

// 声明递归互斥锁
static SemaphoreHandle_t xCanMutex = NULL;

// 分辨率到 RPM/Count 的倒数（Count/RPM）
static const float kResol2RPM[11] = {
    0.0f,   // 占位，索引0不用
    1.0f,   // 1: 1 RPM
    0.5f,   // 2: 0.5 RPM
    1.0f/3, // 3: 1/3 RPM
    0.25f,  // 4: 0.25 RPM
    0.2f,   // 5: 0.2 RPM
    1.0f/6, // 6: 1/6 RPM
    1.0f/7, // 7: 1/7 RPM
    0.125f, // 8: 0.125 RPM
    1.0f/9, // 9: 1/9 RPM
    0.1f    // A: 0.1 RPM
};

// 全局变量：保存当前分辨率档位（1..0xA）
static uint8_t gSpeedResolution = 1;

void CANopen_Init(void) {
    // 注意：调度器启动前不要创建互斥锁
    
    // 初始化 CAN 控制器
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        EnterSafemode(0x51); //can初始化错误
        Error_Handler();
    }

    // 配置发送消息的头部信息
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = std_id; // 确保 std_id 已正确定义
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 8;

    // 配置 FIFO0 的过滤器
    CAN_FilterTypeDef canfilterconfig;
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0; // 选择过滤器组
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到 FIFO0
    canfilterconfig.FilterIdHigh = 0x0000; // 接收所有标准ID的消息
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
        Error_Handler();
    }

    // 配置 FIFO1 的过滤器
    canfilterconfig.FilterBank = 1; // 选择不同的过滤器组
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1; // 分配到 FIFO1
    canfilterconfig.FilterIdHigh = 0x0000; // 接收所有标准ID的消息
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
        Error_Handler();
    }
}

// 安全发送函数（带锁保护）
void Safe_CANopen_Send(uint8_t command, uint16_t index, uint8_t subindex, uint32_t data) {
    // 延迟创建互斥锁（只在调度器运行时创建）
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xCanMutex == NULL) {
        xCanMutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(xCanMutex != NULL);  // 确保创建成功
    }

    // 获取递归锁（支持嵌套调用）
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTakeRecursive(xCanMutex, portMAX_DELAY);
    }

    // 调用原始发送函数
    CANopen_SendSDORequest(command, index, subindex, data);

    // 释放锁
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreGiveRecursive(xCanMutex);
    }
}

void CANopen_SendSDORequest(uint8_t command, uint16_t index, uint8_t subindex, uint32_t data) {
    // 验证锁状态（仅RTOS运行时检查）
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        configASSERT(xSemaphoreGetMutexHolder(xCanMutex) == xTaskGetCurrentTaskHandle());
    }

    uint8_t txData[8];
    uint32_t txMailbox;

    // 命令字
    txData[0] = command;

    // 索引（低位在前，高位在后）
    txData[1] = index & 0xFF;        // 低位
    txData[2] = (index >> 8) & 0xFF; // 高位

    // 子索引
    txData[3] = subindex;

    // 数据（低位在前，高位在后）
    txData[4] = data & 0xFF;         // 最低位
    txData[5] = (data >> 8) & 0xFF;  // 次低位
    txData[6] = (data >> 16) & 0xFF; // 次高位
    txData[7] = (data >> 24) & 0xFF; // 最高位
	
    // 发送消息
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, txData, &txMailbox) != HAL_OK) {
        EnterSafemode(0x52);//can发送错误
        Error_Handler();
    }
    
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // 调度器正在运行，使用FreeRTOS的延时
        vTaskDelay(pdMS_TO_TICKS(1));
    } else {
        HAL_Delay(1);
    }
}

uint8_t CANopen_ReceiveSDOResponse(uint8_t* response) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.StdId == (0x580 + NODE_ID)) {
            for (int i = 0; i < 8; i++) {
                response[i] = rxData[i];
            }
            return 1;
        }
    } else {
       printf("[WARN]Failed to get message from FIFO0\n");
    }
    return 0;
}

// 以下所有函数改为使用Safe_CANopen_Send进行发送操作

//设置电机限速(全局） 最大1000 单位RPM
void  CANopen_SetSpeedLimit(uint16_t speed){
    uint32_t data = (uint32_t)speed;
    Safe_CANopen_Send(SDO_WRITE_2B, 0x2008, 0x00, data);
}

//设置速度模式
void CANopen_SetSpeedMode(void) {
    Safe_CANopen_Send(SDO_WRITE_1B, INDEX_MODE_OF_OPERATION, 0x00, 0x03);
}

//设置速度 根据分辨率换算
void CANopen_SetTargetSpeed(float speedL, float speedR)
{
    float scale = kResol2RPM[gSpeedResolution];   // Count/RPM
    int32_t cntL = (int32_t)(speedL / scale);    // 左侧取反（电机反转）
    int32_t cntR = (int32_t)(- speedR / scale);    // 右侧

    uint32_t data = ((uint32_t)cntR << 16) | ((uint32_t)cntL & 0xFFFF);
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_VELOCITY, 0x03, data);
}

//设置位置模式
void CANopen_SetPositionMode(void){
    Safe_CANopen_Send(SDO_WRITE_1B, INDEX_MODE_OF_OPERATION, 0x00, 0x01);
}

//设置左电机最大速度
void CANopen_SetLeftMaxSpeed(int8_t speed){
    if ( speed > 60) { //设置合理上限 代码支持最大255rpm
        speed=60;
        printf("[WARN]Left speed out of limit! Too high\r\n");
    }else if( speed <=0){
        speed=1;
        printf("[WARN]Left speed out of limit! Too low\r\n");
    }
    
    uint32_t data = (uint32_t)(speed);
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_MAXSPEED, 0x01, data);
}

//设置右电机最大速度
void CANopen_SetRightMaxSpeed(int8_t speed){
    if ( speed > 60) { //设置合理上限 代码支持最大255rpm
        speed=60;
        printf("[WARN]Right speed out of limit! Too high\r\n");
    }else if( speed <=0){
        speed=1;
        printf("[WARN]Right speed out of limit! Too low\r\n");
    }
    
    uint32_t data = (uint32_t)(speed);
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_MAXSPEED, 0x02, data);
}

//设置左电机目标位置
void CANopen_SetLeftPosition(int32_t position){
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_POSITION, 0x01, position);
}

//设置右电机目标位置
void CANopen_SetRightPosition(int32_t position){
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_POSITION, 0x02, position);
}

//启动相对运动
void CANopen_StartRelative(void){
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x4F);
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x5F);
}

//启动绝对运动
void CANopen_StartAbsolute(void){
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x0F);
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x1F);
}

//设置力(转）矩模式
void CANopen_SetTorqueMode(void){
    Safe_CANopen_Send(SDO_WRITE_1B, INDEX_MODE_OF_OPERATION, 0x00, 0x04);
}

//设置左右电机目标转矩（mA/s）
void CANopen_SetTorque(uint32_t left,uint32_t right){
    uint32_t data = (uint32_t)((left << 16) | (right & 0xFFFF));
    Safe_CANopen_Send(SDO_WRITE_4B, INDEX_TARGET_TORQUE, 0x03, data);
}

//电机使能
void CANopen_StartMotor(void) {
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x06);
    //适当增加延时
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x07);
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x0F);
}

// 设置速度分辨率 (索引2026h 子索引05)
/*设置值范围：1-A
1：速度分辨率 1RPM
2：速度分辨率 0.5RPM
3：速度分辨率 1/3RPM
4：速度分辨率 0.25RPM
5：速度分辨率 0.2RPM
6：速度分辨率 1/6RPM
7：速度分辨率 1/7RPM
8：速度分辨率 0.125RPM
9：速度分辨率 1/9RPM
A：速度分辨率 0.1RPM*/
void CANopen_SetSpeedResolution(uint8_t resolution)
{
    if (resolution > 0x0A || resolution < 1) {
        //printf("[WARN]Invalid speed resolution value (1-0xA expected)\n");
        return;
    }
    gSpeedResolution = resolution;                // 记住当前分辨率
    Safe_CANopen_Send(SDO_WRITE_2B, 0x2026, 0x05, resolution);
}

// 设置左电机加速时间 (单位ms)
void CANopen_SetLeftAccelTime(uint16_t time_ms) {
    if (time_ms > 32767) {
        time_ms = 32767;
        //printf("[WARN]Left accel time capped at 32767ms\n");
    }
    Safe_CANopen_Send(SDO_WRITE_4B, 0x6083, 0x01, time_ms);
}

// 设置右电机加速时间 (单位ms)
void CANopen_SetRightAccelTime(uint16_t time_ms) {
    if (time_ms > 32767) {
        time_ms = 32767;
        //printf("[WARN]Right accel time capped at 32767ms\n");
    }
    Safe_CANopen_Send(SDO_WRITE_4B, 0x6083, 0x02, time_ms);
}

// 设置左电机减速时间 (单位ms)
void CANopen_SetLeftDecelTime(uint16_t time_ms) {
    if (time_ms > 32767) {
        time_ms = 32767;
        //printf("[WARN]Left decel time capped at 32767ms\n");
    }
    Safe_CANopen_Send(SDO_WRITE_4B, 0x6084, 0x01, time_ms);
}

// 设置右电机减速时间 (单位ms)
void CANopen_SetRightDecelTime(uint16_t time_ms) {
    if (time_ms > 32767) {
        time_ms = 32767;
        //printf("[WARN]Right decel time capped at 32767ms\n");
    }
    Safe_CANopen_Send(SDO_WRITE_4B, 0x6084, 0x02, time_ms);
}

// 同时设置左右电机加减速时间 (单位ms)
void CANopen_SetMotorAccelDecel(uint16_t accel_time, uint16_t decel_time) {
    CANopen_SetLeftAccelTime(accel_time);
    CANopen_SetRightAccelTime(accel_time);
    CANopen_SetLeftDecelTime(decel_time);
    CANopen_SetRightDecelTime(decel_time);
}

void CANopen_SaveToEEPROM(void) {
    // 发送SDO指令: 2B命令(写2字节), 索引2010h, 子索引00, 数据0x0001
    Safe_CANopen_Send(SDO_WRITE_2B, 0x2010, 0x00, 0x0001);
    
    // 重要：EEPROM写入需要时间，建议延时100ms以上
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        HAL_Delay(100);
    }
}

//停止电机
void CANopen_StopMotor(void) {
    Safe_CANopen_Send(SDO_WRITE_2B, INDEX_CONTROL_WORD, 0x00, 0x00);
}

//心跳报文启动
void CANopen_Heartbeat(void){
    Safe_CANopen_Send(SDO_WRITE_2B, 0x1017, 0x00, 0x03E8);
}

//查看消息队列
uint32_t Checkmailbox(void){
    uint32_t freeMailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    return freeMailboxes;
}

// 更新数据（整体加锁保护）
uint8_t CanOpen_UpdateInfo(void) {
    // 延迟创建互斥锁（只在调度器运行时创建）
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xCanMutex == NULL) {
        xCanMutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(xCanMutex != NULL);  // 确保创建成功
    }
    
    // 进入临界区
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTakeRecursive(xCanMutex, portMAX_DELAY);
    }
    
    uint8_t result = 0;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    // 清空FIFO防止数据错位
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    }

    uint8_t response[8];
    int32_t voltageRaw, currentRaw, speedRaw, targetSpeedRaw, maxSpeedRaw;
    uint32_t statusValue = 0; // 存储状态字
    
    // 将变量声明移到前面，避免被goto跳过初始化
    uint16_t leftStatus = 0;
    uint16_t rightStatus = 0;
    uint8_t eStopPressed = 0;

    // 1. 读取母线电压 (0x2035:00)
    Safe_CANopen_Send(SDO_READ, 0x2035, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    voltageRaw = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
    stat.batteryVoltage = voltageRaw * 0.01; // 单位: 0.01V

    // 2. 读取电机电流 (0x6077:03)
    Safe_CANopen_Send(SDO_READ, 0x6077, 0x03, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    currentRaw = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
    stat.motorCurrentL = ((int16_t)(currentRaw & 0xFFFF)) * 0.1; // 左电机
    stat.motorCurrentR = ((int16_t)((currentRaw >> 16) & 0xFFFF)) * 0.1; // 右电机
    stat.motorCurrent = stat.motorCurrentL + stat.motorCurrentR;

    // 3. 读取实际转速 (0x606C:03)
    Safe_CANopen_Send(SDO_READ, 0x606C, 0x03, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    speedRaw = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
    stat.speedL = (float)((int16_t)(speedRaw & 0xFFFF)) * 0.1; // 左电机
    stat.speedR = - (float)((int16_t)((speedRaw >> 16) & 0xFFFF)) * 0.1; // 右电机

    // 4. 读取目标转速 (0x60FF:03)
    Safe_CANopen_Send(SDO_READ, 0x60FF, 0x03, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    targetSpeedRaw = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
		float scale = kResol2RPM[gSpeedResolution];   // Count/RPM
    stat.req_speedL = (((int16_t)(targetSpeedRaw & 0xFFFF)) * scale);    // 左侧取反（电机反转）
    stat.req_speedR = -(((int16_t)((targetSpeedRaw >> 16))) * scale);    // 右侧
    //stat.req_speedL = (int16_t)(targetSpeedRaw & 0xFFFF); // 左电机
    //stat.req_speedR = (int16_t)((targetSpeedRaw >> 16) & 0xFFFF); // 右电机

    // 5. 读取最大转速 (0x2008:00)
    Safe_CANopen_Send(SDO_READ, 0x2008, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    maxSpeedRaw = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
    stat.speedLimit = (int16_t)maxSpeedRaw;

    // 6. 单次读取状态字（同时获取急停和使能状态）
    Safe_CANopen_Send(SDO_READ, INDEX_STATUS_WORD, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!CANopen_ReceiveSDOResponse(response)) {
        goto cleanup;
    }
    statusValue = (uint32_t)response[4] | 
                 ((uint32_t)response[5] << 8) |
                 ((uint32_t)response[6] << 16) |
                 ((uint32_t)response[7] << 24);

    // 解析状态字 (同时处理左右电机)
    leftStatus = statusValue & 0xFFFF;
    rightStatus = (statusValue >> 16) & 0xFFFF;

    // 检查急停状态 (bit15)
    eStopPressed = (leftStatus & (1 << ESTOP_BIT)) || 
                   (rightStatus & (1 << ESTOP_BIT));
    stat.Ready = !eStopPressed; // Ready与急停状态相反

    // 检查电机使能状态 (低4位=0111)
    stat.motorEN = ((leftStatus & ENABLE_BITS_MASK) == ENABLED_STATE) &&
                   ((rightStatus & ENABLE_BITS_MASK) == ENABLED_STATE);

    result = 1; // 所有信息更新成功

cleanup:
    // 退出临界区
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreGiveRecursive(xCanMutex);
    }
    return result;
}

//接收消息处理
void CANopen_MessageHandler(uint8_t data[8]){
    //@TODO处理返回的数据
}
