#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

#define MPI 3.14159265358979323846f // π

typedef struct
{
    float angle;      // 传感器角度
    float angle_prev; // 上一次读取的角度

    float angle_with_rotations;      // 带圈数的角度
    float angle_with_rotations_prev; // 带圈数的角度

    int32_t full_rotations;      // 完整旋转圈数
    int32_t full_rotations_prev; // 用于速度计算的先前完整旋转圈数

    float velocity;      // 传感器速度
    float velocity_prev; // 上一次用于速度计算的角度

    float loop_ts;      // 上一次读取角度的时间戳
    float loop_ts_prev; // 上一次用于速度计算的角度时间戳

} Sensor;

void Sensor_ReadAngle(Sensor *sensor);
void Sensor_ComputerAngleRotations(Sensor *sensor);
void Sensor_ComputerVelocity(Sensor *sensor);
float Sensor_GetAngle(Sensor *sensor);
float Sensor_GetAngleWithRotations(Sensor *sensor);
float Sensor_GetVelocity(Sensor *sensor);
void Sensor_Update(Sensor *sensor);

#endif

