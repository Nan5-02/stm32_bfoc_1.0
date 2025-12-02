#include "AS5600.h"
#include "i2c.h"

/**************************************************************
 * 函数名：uint8_t AS5600_checkMagnet(void)
 * 函数功能：判断强弱磁铁
 * 输入参数：无
 * 输出参数：1强磁铁 0弱磁铁
 * 返回值：无
 **************************************************************/

uint8_t AS5600_checkMagnet(void)
{
    uint8_t s = 0;
    HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, (uint16_t)_stat, I2C_MEMADD_SIZE_8BIT, &s, 1, 0xff);
    if ((s & 0x20) == 0)
        return 0xFF; // 无磁体
    if (s & 0x08)
        return 2; // 强磁(过强)
    if (s & 0x10)
        return 1; // 弱磁(过弱)
    return 0;     // 正常
}

/**************************************************************
 * 函数名：float AS5600_ReadAngle(void)
 * 函数功能：读取传感器原始数据
 * 输入参数：无
 * 输出参数：有磁铁 传感器原始数据（范围0-4095）  无磁铁 返回上次的数据
 * 返回值：无
 **************************************************************/
float AS5600_ReadAngle(void)
{
    static float data = 0;
    static uint8_t raw_data[2];

    /* Read High Byte */
    HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, (uint16_t)_raw_ang_hi, I2C_MEMADD_SIZE_8BIT, raw_data, 2, 0xff);
    // 将两个八位 合成一个16位
    data = (raw_data[0] & 0x0f) << 8 | raw_data[1];
    /* Raw data reports 0 - 4095 segments */
    if (data < 0)
    {
        data += 4095.0f;
    }
    float angle = data * 2.0f * 3.14159 / 4095.0f;
    return angle;
}
