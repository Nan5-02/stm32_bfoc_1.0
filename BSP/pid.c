#include "pid.h"
#include <main.h>
#include <stdio.h>
#include <dwt_delay.h>
#include "lpd.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

lpd_kalman_t pid_d_lpf;

void PID_SetDerivativeFilterFreq(PID_Controller *pid, float F)
{
    if (F < 0.1f)
        F = 0.1f;
    pid->d_filter_rc = 1.0f / (2.0f * PI * F);
    // 初始化滤波输出
    pid->d_filtered = 0.0f;
}

void PID_SetOutputFilterFreq(PID_Controller *pid, float F)
{
    if (F < 0.1f)
        F = 0.1f;
    pid->output_filter_rc = 1.0f / (2.0f * PI * F);
    // 初始化滤波输出
    pid->output_filtered = 0.0f;
}

void PID_SetInputFilterFreq(PID_Controller *pid, float F)
{
    if (F < 0.1f)
        F = 0.1f;
    pid->input_filter_rc = 1.0f / (2.0f * PI * F);
    // 初始化滤波输出
    pid->input_filtered = 0.0f;
}

void fittered_init(void)
{
    LPF_Kalman_Init(&pid_d_lpf, 0.01f, 0.5f);
}

void PidCompute(PID_Controller *pid, float measurement)
{
    static uint8_t first_run = 0;
    /*----------------------------------计算时间戳------------------------------------*/
    uint32_t now_us = DWT_Get_Microsecond();
    float dt = (pid->last_time_us == 0) ? 0.002f : (now_us - pid->last_time_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.001f;

    pid->last_time_us = now_us;
    if (first_run == 0)
    {
        if (measurement - pid->setpoint < 0.1f && measurement - pid->setpoint > -0.1f)
        {
            first_run = 1;
        }
    }

    /*----------------------------------PID计算------------------------------------*/
    // 对输入进行低通滤波（若未配置频率则默认 20Hz）
    if (pid->input_filter_rc <= 0.0f)
        pid->input_filter_rc = 1.0f / (2.0f * PI * 100.0f);

    float a_in = dt / (pid->input_filter_rc + dt);
    pid->input_filtered = a_in * measurement + (1.0f - a_in) * pid->input_filtered;

    // 使用滤波后的输入计算误差
    float error = pid->setpoint - pid->input_filtered;
    // 积分项累加
    pid->integral += error;
    // 原始微分
    float raw_d = (error - pid->previous_error) / dt;

#if 0
    pid->d_filtered = raw_d; // 暂时不使用滤波
#else
    pid->d_filtered = LPF_Kalman_Update(&pid_d_lpf, raw_d);
#endif

    // 更新上一次误差
    pid->previous_error = error;

    // 计算PID原始输出
    float raw_output = (error * pid->Kp + pid->integral * pid->Ki + pid->d_filtered * pid->Kd) * pid->direction; // 计算PID原始输出

    // 对输出进行低通滤波（若未配置频率则默认 200Hz）
    //    if (pid->output_filter_rc <= 0.0f)
    //        pid->output_filter_rc = 1.0f / (2.0f * PI * 200.0f);
    //    float a_out = dt / (pid->output_filter_rc + dt);
    //    pid->output_filtered = a_out * raw_output + (1.0f - a_out) * pid->output_filtered;

    // 使用滤波后的输出
    pid->output = raw_output;

    if (first_run == 0)
    {
        if (pid->output >= 6.0f)
            pid->output = 6.0f;
        else if (pid->output <= -6.0f)
            pid->output = -6.0f;
    }
}
