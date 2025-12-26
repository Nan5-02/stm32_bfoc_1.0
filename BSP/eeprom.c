#include "eeprom.h"
#include <string.h>

/* 内部常量 */
#define EEPROM_MAGIC (0xEEB0AA55UL)
#define EEPROM_VERSION (0x0001U)

/* 页起始处存储的紧凑头部 */
#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;    /* 魔数：0xEEB0AA55 */
    uint16_t version;  /* 结构/版本标签 */
    uint16_t length;   /* 有效载荷字节长度 */
    uint16_t crc16;    /* 有效载荷的 CRC-16/CCITT */
    uint16_t reserved; /* 对齐至总计 12 字节 */
} eeprom_header_t;
#pragma pack(pop)

/* 初始化后缓存的元数据 */
static uint8_t s_valid = 0;
static uint16_t s_len = 0;

/* 前置声明 */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
static HAL_StatusTypeDef flash_page_erase(uint32_t page_addr);
static HAL_StatusTypeDef flash_write_halfword_stream(uint32_t addr, const uint8_t *buf, uint32_t len);

void EEPROM_Init(void)
{
    const eeprom_header_t *hdr = (const eeprom_header_t *)EEPROM_FLASH_PAGE_ADDRESS;

    s_valid = 0;
    s_len = 0;
    if (hdr->magic != EEPROM_MAGIC)
    {
        return; /* 空或无效 */
    }

    if (hdr->version != EEPROM_VERSION)
    {
        return; /* 版本不兼容 */
    }

    if (hdr->length == 0 || hdr->length > (EEPROM_FLASH_PAGE_SIZE - sizeof(eeprom_header_t)))
    {
        return; /* 长度不合法 */
    }

    const uint8_t *payload = (const uint8_t *)(EEPROM_FLASH_PAGE_ADDRESS + sizeof(eeprom_header_t));
    uint16_t calc = crc16_ccitt(payload, hdr->length);
    if (calc != hdr->crc16)
    {
        return; /* 数据损坏 */
    }

    s_valid = 1;
    s_len = hdr->length;
}

uint8_t EEPROM_IsValid(void)
{
    return s_valid;
}

uint16_t EEPROM_GetStoredLength(void)
{
    return s_len;
}

HAL_StatusTypeDef EEPROM_Read(void *dst, uint16_t len)
{
    if (!s_valid || len == 0 || len != s_len)
    {
        return HAL_ERROR;
    }
    const uint8_t *payload = (const uint8_t *)(EEPROM_FLASH_PAGE_ADDRESS + sizeof(eeprom_header_t));
    memcpy(dst, payload, len);
    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_Write(const void *src, uint16_t len)
{
    if (src == NULL || len == 0 || len > (EEPROM_FLASH_PAGE_SIZE - sizeof(eeprom_header_t)))
    {
        return HAL_ERROR;
    }

    eeprom_header_t hdr;
    hdr.magic = EEPROM_MAGIC;
    hdr.version = EEPROM_VERSION;
    hdr.length = len;
    hdr.crc16 = crc16_ccitt((const uint8_t *)src, len);
    hdr.reserved = 0xFFFFU; /* not used */

    /* 擦除目标页 */
    if (flash_page_erase(EEPROM_FLASH_PAGE_ADDRESS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 写入头部 */
    if (flash_write_halfword_stream(EEPROM_FLASH_PAGE_ADDRESS, (const uint8_t *)&hdr, sizeof(hdr)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 写入有效载荷 */
    uint32_t payload_addr = EEPROM_FLASH_PAGE_ADDRESS + sizeof(eeprom_header_t);
    if (flash_write_halfword_stream(payload_addr, (const uint8_t *)src, len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 更新缓存 */
    s_valid = 1;
    s_len = len;
    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_Clear(void)
{
    if (flash_page_erase(EEPROM_FLASH_PAGE_ADDRESS) != HAL_OK)
    {
        return HAL_ERROR;
    }
    s_valid = 0;
    s_len = 0;
    return HAL_OK;
}

/* ---- 内部辅助函数 ---- */

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x8000U)
            {
                crc = (crc << 1) ^ 0x1021U;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static HAL_StatusTypeDef flash_page_erase(uint32_t page_addr)
{
    HAL_StatusTypeDef st;
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef erase = {0};

    HAL_FLASH_Unlock();

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = page_addr;
    erase.NbPages = 1;

    st = HAL_FLASHEx_Erase(&erase, &page_error);

    HAL_FLASH_Lock();

    if (st != HAL_OK || page_error != 0xFFFFFFFFUL)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef flash_write_halfword_stream(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    HAL_StatusTypeDef st = HAL_OK;

    HAL_FLASH_Unlock();

    /* 按 16 位半字编程；奇数字节长度以 0xFF 填充 */
    uint32_t i = 0;
    while (i < len)
    {
        uint16_t hw = buf[i];
        if ((i + 1) < len)
        {
            hw |= ((uint16_t)buf[i + 1]) << 8;
        }
        else
        {
            hw |= 0xFF00U; /* 填充 */
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, hw) != HAL_OK)
        {
            st = HAL_ERROR;
            break;
        }

        /* 校验 */
        if (*(volatile uint16_t *)addr != hw)
        {
            st = HAL_ERROR;
            break;
        }

        addr += 2U;
        i += 2U;
    }

    HAL_FLASH_Lock();
    return st;
}
