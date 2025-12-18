#ifndef __MYCAN_H
#define __MYCAN_H

#include <can.h>
// 初始化CAN外设
void MyCan_Init(void);
// 设置CAN滤波器
HAL_StatusTypeDef MyCan_SetupFilter(void);
// 发送CAN消息
HAL_StatusTypeDef MyCan_Transmit(uint32_t StdId, uint8_t *data, uint8_t len);
// 接收CAN消息
HAL_StatusTypeDef MyCan_Receive(CAN_RxHeaderTypeDef *rxHeader, uint8_t *data);
// 处理接收到的CAN消息
void MyCan_ProcessReceivedMessage(void);
// 获取CAN运行状态标志
uint8_t getCanRunFlag(void);
// 获取IMU数据接收标志
uint8_t getImuDataFlag(void);
// 清除IMU数据接收标志
void clealImuDataFlag(void);

#endif
