#include "modbus.h"
#include "usart.h"
#include <string.h>
#include "pid.h"
#include "FOC.h"
#include <main.h>

uint8_t uart1_rx_buf[100];
uint16_t uart1_rx_size;

uint8_t rx_state = 0;
uint8_t frame_acc[100];
uint16_t frame_acc_len = 0;
uint8_t collecting = 0;

uint8_t motor_start_flag = 0;

extern FOC motor_foc;

void modbus_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, sizeof(uart1_rx_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // 关半传输中断，避免两次回调
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        rx_state = 1;
        uart1_rx_size = Size;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

uint8_t modbus_get_motor_start_flag(void)
{
    return motor_start_flag;
}

uint8_t crc_buffer[30];
static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

uint8_t modbus_progress(void)
{
    if (rx_state == 0)
        return 0; // 无新数据
    for (uint16_t i = 0; i < uart1_rx_size; ++i)
    {
        uint8_t b = uart1_rx_buf[i];

        if (!collecting)
        {
            // 等待帧头
            if (b == FRAME_HEAD)
            {
                collecting = 1;
                frame_acc_len = 0;
                frame_acc[frame_acc_len++] = b;
            }
            continue;
        }
        // 判断重复的帧头
        if (b == FRAME_HEAD)
        {
            frame_acc_len = 0;
            frame_acc[frame_acc_len++] = b;
            continue;
        }
        // 判断帧尾
        if (b == FRAME_TAIL)
        {
            rx_state = 0;
            collecting = 0;
            frame_acc_len = 0;
        }
        else
        {
            frame_acc[frame_acc_len++] = b;
        }
    }

    // 继续ToIdle接收（部分HAL会持续，这里显式重启更保险）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, sizeof(uart1_rx_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    return 1; // 有新数据
}

void modbus_update(void)
{

    // 处理modbus数据
    if (frame_acc[0] == 0xAA) // 从机地址
    {
        /*-------------------------------------写指令 -------------------------------*/
        if (frame_acc[1] == 0x01)
        {
            /*------------------启动 ----------------------*/
            if (frame_acc[2] == 0x01)
            {
                if (frame_acc[3] == 0x00)
                {
                    // 启动电机
                    motor_start_flag = 1;
                }
            }
            /*------------------复位 ----------------------*/
            else if (frame_acc[2] == 0x02)
            {
                if (frame_acc[3] == 0x00)
                {
                    // 复位电机
                    NVIC_SystemReset();
                }
            }
            /*------------------设置电机角度目标值 float ----------------------*/
            else if (frame_acc[2] == 0x03)
            {

                uint16_t crc = Modbus_CRC16(frame_acc, 7);
                uint16_t received_crc = ((uint16_t)frame_acc[8] << 8) | frame_acc[7];
                if (crc != received_crc)
                {
                    // CRC校验失败，丢弃数据
                    return;
                }

                union
                {
                    uint32_t u;
                    float f;
                } conv;

                uint8_t b0 = frame_acc[3];
                uint8_t b1 = frame_acc[4];
                uint8_t b2 = frame_acc[5];
                uint8_t b3 = frame_acc[6];
                conv.u = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | ((uint32_t)b3);
                Motor_SetTarget(&motor_foc, conv.f);
                // printf("uq set: %f\r\n", motor_foc.Uq);
            }
            /*------------------设置电机控制器 PID ----------------------*/
            else if (frame_acc[2] == 0x04)
            {
                PID_Controller pid_data;

                union
                {
                    uint32_t u;
                    float f;
                } Kp, Ki, Kd;

                Kp.u = ((uint32_t)frame_acc[3] << 24) | ((uint32_t)frame_acc[4] << 16) | ((uint32_t)frame_acc[5] << 8) | ((uint32_t)frame_acc[6]);
                Ki.u = ((uint32_t)frame_acc[7] << 24) | ((uint32_t)frame_acc[8] << 16) | ((uint32_t)frame_acc[9] << 8) | ((uint32_t)frame_acc[10]);
                Kd.u = ((uint32_t)frame_acc[11] << 24) | ((uint32_t)frame_acc[12] << 16) | ((uint32_t)frame_acc[13] << 8) | ((uint32_t)frame_acc[14]);

                pid_data.Kp = Kp.f;
                pid_data.Ki = Ki.f;
                pid_data.Kd = Kd.f;
                // printf("Position PID set: Kp=%f, Ki=%f, Kd=%f\r\n", pid_data.Kp, pid_data.Ki, pid_data.Kd);
                Motor_SetPositionPID(&motor_foc, pid_data);
            }

            /*------------------设置电机速度 PID ----------------------*/
            else if (frame_acc[2] == 0x05)
            {
                PID_Controller pid_data;
                union
                {
                    uint32_t u;
                    float f;
                } velocity_Kp, velocity_Ki, velocity_Kd;

                velocity_Kp.u = ((uint32_t)frame_acc[3] << 24) | ((uint32_t)frame_acc[4] << 16) | ((uint32_t)frame_acc[5] << 8) | ((uint32_t)frame_acc[6]);
                velocity_Ki.u = ((uint32_t)frame_acc[7] << 24) | ((uint32_t)frame_acc[8] << 16) | ((uint32_t)frame_acc[9] << 8) | ((uint32_t)frame_acc[10]);
                velocity_Kd.u = ((uint32_t)frame_acc[11] << 24) | ((uint32_t)frame_acc[12] << 16) | ((uint32_t)frame_acc[13] << 8) | ((uint32_t)frame_acc[14]);
                pid_data.Kp = velocity_Kp.f;
                pid_data.Ki = velocity_Ki.f;
                pid_data.Kd = velocity_Kd.f;
                Motor_SetVelocityPID(&motor_foc, pid_data);
            }
            /*------------------设置 FOC 模式 ----------------------*/
            else if (frame_acc[2] == 0x06)
            {
                Motor_SetFocControlMode(&motor_foc, frame_acc[3]);
            }
            /*------------------设置 FOC 方向 ----------------------*/
            else if (frame_acc[2] == 0x07)
            {
                Motor_SetFocDirection(&motor_foc, frame_acc[3]);
            }
            /*------------------设置极对数 ----------------------*/
            else if (frame_acc[2] == 0x08)
            {
                Motor_SetElectricalpolePairs(&motor_foc, frame_acc[3]);
            }
        }
        /*-------------------------------------读指令 --------------------------------------*/
        else if (frame_acc[1] == 0x02)
        {
            /*------------------AS5600 磁场状态 ----------------------*/
            if (frame_acc[2] == 0x01)
            {
            }
        }
    }
}

void modbus_loop(void)
{
    if (modbus_progress())
    {
        modbus_update();
    }
}
