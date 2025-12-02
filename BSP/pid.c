#include "pid.h"
#include <main.h>
#include <stdio.h>
#include <dwt_delay.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void PID_SetDerivativeFilterFreq(PID_Controller *pid, float F)
{
    if (F < 0.1f)
        F = 0.1f;
    pid->d_filter_rc = 1.0f / (2.0f * PI * F);
    // 初始化滤波输出
    pid->d_filtered = 0.0f;
}

void PidCompute(PID_Controller *pid, float measurement)
{
    // printf("%f\r\n", measurement);
    static uint8_t count = 0;

    uint32_t now_us = DWT_Get_Microsecond();
    float dt = (pid->last_time_us == 0) ? 0.002f : (now_us - pid->last_time_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.001f;

    pid->last_time_us = now_us;
    float error = pid->setpoint - measurement; // 计算误差

    pid->integral += error; // 积分项累加

    // 原始微分
    float raw_d = (error - pid->previous_error) / dt;
    // 低通滤波 a = dt/(RC + dt)，若未配置频率则给默认 100Hz
    if (pid->d_filter_rc <= 0.0f)
        pid->d_filter_rc = 1.0f / (2.0f * PI * 20.0f);
    float a = dt / (pid->d_filter_rc + dt);

    pid->d_filtered = a * raw_d + (1.0f - a) * pid->d_filtered;

    pid->previous_error = error; // 更新上一次误差

    pid->d_filtered_last = pid->d_filtered;
    pid->d_filtered_last2 = pid->d_filtered_last;

    if (count > 3)
    {
        pid->d_filtered = (pid->d_filtered + pid->d_filtered_last + pid->d_filtered_last2) / 3.0f;
        // printf("%f,%f,%f\r\n", raw_d, pid->d_filtered, dt);
    }
    else
    {
        count++;
    }

    pid->output = (error * pid->Kp + pid->integral * pid->Ki + pid->d_filtered * pid->Kd) * pid->direction; // 计算PID输出
    // printf("PID Compute: error=%f, integral=%f, d_error=%f,output:%f\r\n", error, pid->integral, d_error, pid->output);
}

void PidComputeUq(PID_Controller *pid)
{
    float error = pid->setpoint;                 // 计算误差
    float d_error = error - pid->previous_error; // 计算微分项

    if (d_error == 0) // 死区处理
    {
        return;
    }

    pid->integral += error;      // 积分项累加
    pid->previous_error = error; // 更新上一次误差

    pid->output = (error * pid->Kp + pid->integral * pid->Ki + d_error * pid->Kd) * pid->direction; // 计算PID输出
}
