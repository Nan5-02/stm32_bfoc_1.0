#include "AS5600.h"
#include "i2c.h"
#include <math.h>
#include "sensor.h"

void Sensor_ReadAngle(Sensor *sensor)
{
    sensor->angle = AS5600_ReadAngle();
}

void Sensor_ComputerAngleRotations(Sensor *sensor)
{
    static uint8_t initialized = 0; // 初始化标志

    Sensor_ReadAngle(sensor); // 读取当前角度

    // 首次初始化
    if (!initialized)
    {
        sensor->angle_prev = sensor->angle;
        initialized = 1;
        return;
    }

    float angle_diff = sensor->angle - sensor->angle_prev; // 计算与上一次角度的差值

    // 判断是否跨圈（超过0.8圈 ≈ 5.03弧度），修正圈数
    if (fabsf(angle_diff) > (0.8f * 2.0f * MPI))
    {
        sensor->full_rotations += (angle_diff > 0) ? -1 : 1;
    }

    sensor->angle_prev = sensor->angle; // 更新上一次角度值

    // 返回总角度（圈数*2π + 当前角度）
    sensor->angle_with_rotations = (float)sensor->full_rotations * 2.0f * MPI + sensor->angle;
}

void Sensor_ComputerVelocity(Sensor *sensor)
{
    float Ts;

    Sensor_ComputerAngleRotations(sensor);                 // 计算带圈数的角度
    sensor->loop_ts = DWT_Get_Microsecond();               // 更新时间戳
    Ts = (sensor->loop_ts - sensor->loop_ts_prev) * 1e-6f; // 计算时间差，单位秒
    if (Ts <= 0)
        Ts = 1e-3f;                                                                             // 防止除零
    sensor->velocity = (sensor->angle_with_rotations - sensor->angle_with_rotations_prev) / Ts; // 计算速度
    sensor->angle_with_rotations_prev = sensor->angle_with_rotations;                           // 更新上一次带圈数的角度
    sensor->loop_ts_prev = sensor->loop_ts;                                                     // 更新上一次时间戳
}

float Sensor_GetAngle(Sensor *sensor)
{
    return sensor->angle;
}

float Sensor_GetAngleWithRotations(Sensor *sensor)
{
    return sensor->angle_with_rotations;
}

float Sensor_GetVelocity(Sensor *sensor)
{
    return sensor->velocity;
}

void Sensor_Update(Sensor *sensor)
{
    Sensor_ComputerVelocity(sensor);
}
