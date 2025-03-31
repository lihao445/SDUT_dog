#include "trajectory.h"
#include <math.h>

// ����ƽ�������������ȿɵ���
float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y = number;
    i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // һ��ţ�ٵ���
    return y;
}

// ���˶�ѧ������x,y�����theta1��theta2����λ�����ȣ�
void InverseKinematics(float x, float y, float *theta1, float *theta2) {
    float L_sq = x*x + y*y;
    float L = 1.0f / Q_rsqrt(L_sq); // ����ƽ����
    float numerator = L_sq + L1*L1 - L2*L2;
    float denominator = 2.0f * L1 * L;
    float cos_beta = numerator / denominator;
    
    // ����cos_beta��[-1,1]��Χ��
    cos_beta = (cos_beta > 1.0f) ? 1.0f : (cos_beta < -1.0f) ? -1.0f : cos_beta;
    float beta = acosf(cos_beta);
    float phi = atan2f(y, x);
    
    *theta1 = phi - beta+M_PI;
    *theta2 = phi + beta;
    
}