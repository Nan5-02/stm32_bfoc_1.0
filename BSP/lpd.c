#include "lpd.h"

// 初始化：设置过程噪声 q、测量噪声 r，清零状态
void LPF_Kalman_Init(lpd_kalman_t *kf, float q, float r)
{
    kf->q = q;
    kf->r = r;
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

// 复位：保持噪声参数，重置状态与协方差
void LPF_Kalman_Reset(lpd_kalman_t *kf)
{
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

// 更新：输入一次测量，返回滤波后的估计
float LPF_Kalman_Update(lpd_kalman_t *kf, float z)
{
    // 预测
    float x_pred = kf->x;
    float p_pred = kf->p + kf->q;

    // 更新
    kf->k = p_pred / (p_pred + kf->r);
    kf->x = x_pred + kf->k * (z - x_pred);
    kf->p = (1.0f - kf->k) * p_pred;

    return kf->x;
}
