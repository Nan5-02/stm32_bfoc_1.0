#ifndef __PID_H
#define __PID_H

// #include "FOC.h"
#include <stdint.h>

typedef struct
{
    float Kp;             // Proportional gain
    float Ki;             // Integral gain
    float Kd;             // Derivative gain
    float setpoint;       // Desired target value
    float integral;       // Integral term
    float previous_error; // Previous error value
    float output;         // PID output
    int direction;        // Control direction (1 for direct, -1 for reverse)

    float d_filtered;       // 低通滤波后的微分
    float d_filtered_last;  // 微分滤波频率
    float d_filtered_last2; // 微分滤波频率

    float d_filter_rc;     // RC 常数 = 1/(2πF)
    uint32_t last_time_us; // 上次时间戳(微秒)

    float output_filtered;  // 低通滤波后的输出
    float output_filter_rc; // 输出滤波器 RC 常数 = 1/(2πF)

    float input_filtered;  // 低通滤波后的输入
    float input_filter_rc; // 输入滤波器 RC 常数 = 1/(2πF)

} PID_Controller;

void PidCompute(PID_Controller *pid, float measurement);
void PID_SetDerivativeFilterFreq(PID_Controller *pid, float F);
void PID_SetOutputFilterFreq(PID_Controller *pid, float F);
void PID_SetInputFilterFreq(PID_Controller *pid, float F);
void fittered_init(void);

#endif
