#ifndef __LPD_H
#define __LPD_H

// 简单一阶卡尔曼滤波器
typedef struct
{
    float q; // 过程噪声方差
    float r; // 测量噪声方差
    float x; // 状态估计
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} lpd_kalman_t;

void LPF_Kalman_Init(lpd_kalman_t *kf, float q, float r);
float LPF_Kalman_Update(lpd_kalman_t *kf, float measurement);
void LPF_Kalman_Reset(lpd_kalman_t *kf);

#endif // !__LPD_H
