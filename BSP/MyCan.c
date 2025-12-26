#include "MyCan.h"
#include <can.h>
#include <stdio.h>
#include <string.h>
#include "FOC.h"

CAN_RxHeaderTypeDef can_rxHeader;
uint8_t can_rx_data[8];
uint8_t can_rx_flag = 0;

extern FOC motor_foc;

HAL_StatusTypeDef MyCan_SetupFilter(void)
{
    CAN_FilterTypeDef canFilter;

    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;                          // 使用滤波器0
    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到FIFO0
    canFilter.FilterIdHigh = 0x300 << 5;               // 标准ID左移5位
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x7E0 << 5; // 掩码，匹配所有标准ID
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK; // 列表模式
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.SlaveStartFilterBank = 13; // 主CAN使用前13个滤波器

    return HAL_CAN_ConfigFilter(&hcan, &canFilter);
}

// 初始化CAN外设
void MyCan_Init(void)
{
    if (MyCan_SetupFilter() != HAL_OK)
    {
        printf("CAN filter config failed, err=0x%08lX\r\n", HAL_CAN_GetError(&hcan));
        return;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        printf("CAN start failed, state=%u, err=0x%08lX\r\n", hcan.State, HAL_CAN_GetError(&hcan));
        return;
    }
    // 使能接收中断
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("CAN activate notification failed, err=0x%08lX\r\n", HAL_CAN_GetError(&hcan));
        return;
    }
}

// 设置CAN滤波器

// 发送CAN消息
HAL_StatusTypeDef MyCan_Transmit(uint32_t StdId, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.StdId = StdId;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = len;
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox);
}

// 接收CAN消息
HAL_StatusTypeDef MyCan_Receive(CAN_RxHeaderTypeDef *rxHeader, uint8_t *data)
{
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, rxHeader, data) != HAL_OK)
    {
        return HAL_ERROR; // 接收失败
    }

    return HAL_OK; // 接收成功
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // 接收到消息时切换LED状态
    if (MyCan_Receive(&can_rxHeader, can_rx_data) == HAL_OK)
    {
        can_rx_flag = 1; // 设置接收标志
    }
}

// 电机运行
uint8_t can_run_flag = 0;
uint8_t getCanRunFlag(void)
{
    return can_run_flag;
}

uint8_t imu_data_flag;
uint8_t getImuDataFlag(void)
{
    return imu_data_flag;
}
void clealImuDataFlag(void)
{
    imu_data_flag = 0;
}

void MyCan_ProcessReceivedMessage(void)
{
    if (can_rx_flag)
    {
        // printf("id:%03X,ide:%d\r\n", can_rxHeader.StdId, can_rxHeader.RTR);
        // 处理接收到的数据

        // 启动电机 0x301
        if (can_rxHeader.StdId == 0x301)
        {
            can_run_flag = 1; // 假设第一个字节表示运行状态
        }
        // 复位  0x302
        else if (can_rxHeader.StdId == 0x302)
        {
            HAL_NVIC_SystemReset();
        }
        // 姿态角 0x303
        else if (can_rxHeader.StdId == 0x303)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // 接收到IMU数据时切换LED状态
            float pitch;
            memcpy(&pitch, &can_rx_data[0], 4);
            if (pitch < 0)
                pitch = 360.0f + pitch;
            pitch = (pitch + 0.0) * 3.1415926f / 180.0f; // 转为弧度
            if (motor_foc.control_target == 0)
            {
                motor_foc.Uq = pitch;
            }
            else if (motor_foc.control_target == 1)
            {
                motor_foc.position_pid.setpoint = pitch;
            }
            else if (motor_foc.control_target == 2)
            {
                motor_foc.velocity_pid.setpoint = pitch;
            }
            else if (motor_foc.control_target == 3)
            {
                motor_foc.imu_data = pitch;
            }
            imu_data_flag = 1;
        }
        // 电机控制模式
        else if (can_rxHeader.StdId == 0x304)
        {
            uint8_t control_mode = can_rx_data[0];
            motor_foc.control_target = control_mode;
        }
        // PID P值
        else if (can_rxHeader.StdId == 0x305)
        {
            float Kp;
            memcpy(&Kp, &can_rx_data[0], 4);
            if (motor_foc.control_target == 1)
            {
                motor_foc.position_pid.Kp = Kp;
            }
            else if (motor_foc.control_target == 2)
            {
                motor_foc.velocity_pid.Kp = Kp;
            }
            else if (motor_foc.control_target == 3)
            {
                motor_foc.imu_pid.Kp = Kp;
            }
        }
        // PID I值
        else if (can_rxHeader.StdId == 0x306)
        {
            float Ki;
            memcpy(&Ki, &can_rx_data[0], 4);
            if (motor_foc.control_target == 1)
            {
                motor_foc.position_pid.Ki = Ki;
            }
            else if (motor_foc.control_target == 2)
            {
                motor_foc.velocity_pid.Ki = Ki;
            }
            else if (motor_foc.control_target == 3)
            {
                motor_foc.imu_pid.Ki = Ki;
            }
        }
        // PID D值
        else if (can_rxHeader.StdId == 0x307)
        {
            float Kd;
            memcpy(&Kd, &can_rx_data[0], 4);
            if (motor_foc.control_target == 1)
            {
                motor_foc.position_pid.Kd = Kd;
            }
            else if (motor_foc.control_target == 2)
            {
                motor_foc.velocity_pid.Kd = Kd;
            }
            else if (motor_foc.control_target == 3)
            {

                motor_foc.imu_pid.Kd = Kd;
            }
        }
        // 设置PID方向
        else if (can_rxHeader.StdId == 0x308)
        {
            uint8_t dir = can_rx_data[0];
            if (motor_foc.control_target == 1)
            {
                motor_foc.position_pid.direction = (dir == 0) ? 1 : -1;
            }
            else if (motor_foc.control_target == 2)
            {
                motor_foc.velocity_pid.direction = (dir == 0) ? 1 : -1;
            }
            else if (motor_foc.control_target == 3)
            {
                motor_foc.imu_pid.direction = (dir == 0) ? 1 : -1;
            }
        }
        // 极对数
        else if (can_rxHeader.StdId == 0x309)
        {
            uint8_t pole_pairs = can_rx_data[0];
            motor_foc.pole_pairs = pole_pairs;
        }

        can_rx_flag = 0; // 清除接收标志
    }
}
