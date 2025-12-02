#ifndef __FOC_H
#define __FOC_H

#include "pid.h"
#include "sensor.h"
#include "tim.h"
#include <math.h>
#include "sensor.h"
#include <stdio.h>
#include <dwt_delay.h>
#include <stdio.h>

#define TIM_PORT htim3
#define TIM_CHA TIM_CHANNEL_2
#define TIM_CHB TIM_CHANNEL_3
#define TIM_CHC TIM_CHANNEL_4

typedef struct
{
    TIM_HandleTypeDef port;
    unsigned int channelA;
    unsigned int channelB;
    unsigned int channelC;
} Tim;

typedef struct FOC
{
    float voltage_power_supply; // 电源电压
    float voltage_limit;        // 电压限制
    float voltage_sensor_align; // 传感器对齐电压
    float zero_electric_angle;  // 零电气角
    int pole_pairs;             // 极对数
    int dir;                    // 旋转方向
    uint8_t control_target;     // FOC控制模式

    float Uq;       // q轴电压
    float angle_el; // 电角度

    float target_angle;    // 目标位置
    float target_velocity; // 目标速度

    // 编码器传感器
    Sensor sensor;

    Tim tim;

    // PID参数
    PID_Controller position_pid;
    PID_Controller velocity_pid;

} FOC;

// extern FOC motor_foc;

#define _3PI_2 4.71238898038f        // 3π/2
#define MPI 3.14159265358979323846f  // π
#define M2PI 6.28318530717958647692f // 2π

float _electricalAngle(struct FOC *foc);
void setPhaseVoltage(struct FOC *foc);

void openLoopSpeedControl(struct FOC *foc);

void foc_Init(struct FOC *foc);
void Pidloop(FOC *foc);
void foc_loop(FOC *foc);
void foc_move(FOC *foc);

void Motor_SetTarget(FOC *foc, float target);
void Motor_SetTargetAngle(FOC *foc, float angle);
void Motor_SetTargetVelocity(FOC *foc, float velocity);

void Motor_SetFocControlMode(FOC *foc, uint8_t mode);
void Motor_SetFocDirection(FOC *foc, uint8_t dir);
void Motor_SetElectricalpolePairs(FOC *foc, uint8_t pole_pairs);

void Motor_SetPositionPID(FOC *foc, PID_Controller pid);
void Motor_SetVelocityPID(FOC *foc, PID_Controller pid);

void Motor_LinkTim(FOC *foc, Tim *tim);
void Motor_LinkSensor(FOC *foc, Sensor *sensor);
void Motor_LinkPositionPID(FOC *foc, PID_Controller *pid);
void Motor_LinkVelocityPID(FOC *foc, PID_Controller *pid);

#endif
