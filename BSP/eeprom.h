#ifndef BSP_EEPROM_H
#define BSP_EEPROM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 配置（可在包含此头文件前覆盖以下宏） */
#ifndef EEPROM_FLASH_PAGE_ADDRESS
/* 对于 STM32F103C8（64KB Flash），默认使用最后一个 1KB 页 */
#define EEPROM_FLASH_PAGE_ADDRESS (0x0800FC00UL)
#endif

#ifndef EEPROM_FLASH_PAGE_SIZE
/* STM32F103 中密度器件的页大小为 1KB */
#define EEPROM_FLASH_PAGE_SIZE (1024UL)
#endif

    /* 公开 API */

    /** 初始化 EEPROM 仿真：通过头部与 CRC 校验已存数据是否有效 */
    void EEPROM_Init(void);

    /** 若 Flash 中存在有效数据块返回 1，否则返回 0 */
    uint8_t EEPROM_IsValid(void);

    /** 返回已存数据块长度（字节），无效时返回 0 */
    uint16_t EEPROM_GetStoredLength(void);

    /**
     * 读取已存数据到 dst。
     * - len 必须与已存长度一致；可先调用 EEPROM_GetStoredLength()。
     * - 成功返回 HAL_OK，否则返回 HAL_ERROR。
     */
    HAL_StatusTypeDef EEPROM_Read(void *dst, uint16_t len);

    /**
     * 将数据块写入 Flash 并保存 CRC16。
     * - 先擦除目标页，再写入头部与有效载荷。
     * - 成功返回 HAL_OK，否则返回 HAL_ERROR。
     */
    HAL_StatusTypeDef EEPROM_Write(const void *src, uint16_t len);

    /** 擦除 EEPROM 页（标记为空） */
    HAL_StatusTypeDef EEPROM_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_EEPROM_H */
