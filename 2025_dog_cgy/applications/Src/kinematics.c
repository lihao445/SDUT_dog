#include "trajectory.h"
#include <math.h>

// 快速平方根倒数（精度可调）
float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y = number;
    i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // 一次牛顿迭代
    return y;
}

// 逆运动学（输入x,y，输出theta1和theta2，单位：弧度）
void InverseKinematics(float x, float y, float *theta1, float *theta2) {
    float L_sq = x*x + y*y;
    float L = 1.0f / Q_rsqrt(L_sq); // 快速平方根
    float numerator = L_sq + L1*L1 - L2*L2;
    float denominator = 2.0f * L1 * L;
    float cos_beta = numerator / denominator;
    
    // 限制cos_beta在[-1,1]范围内
    cos_beta = (cos_beta > 1.0f) ? 1.0f : (cos_beta < -1.0f) ? -1.0f : cos_beta;
    float beta = acosf(cos_beta);
    float phi = atan2f(y, x);
    
    *theta1 = phi - beta+M_PI;
    *theta2 = phi + beta;
    
}