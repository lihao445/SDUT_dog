#ifndef _POSTURE_CTRL_H_
#define _POSTURE_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "startup_main.h"
	
#ifdef __cplusplus
}

#include "bsp_motor.h"

#define M_PI 3.1415926f

// 机械参数
#define L1                 150.0f   // 大腿长度 (mm)
#define L2                 300.0f   // 小腿长度 (mm)

#define True 1
#define Flase 0

typedef struct  {		// 腿部参数结构体
    float stance_height ; // 狗身到地面的距离 (mm)
    float step_length ; // 一步的距离 (mm)
    float lift_height; // 抬腿高度 (mm)
    float flight_percent ; // 摆动相百分比
    float freq ; // 一步的频率 (Hz)
} GaitParams;
extern GaitParams gait_params; // 当前腿部参数
extern GaitParams state_gait_params[]; // 不同状态下的参数设置

typedef struct {

    GaitParams detached_params_0;
    GaitParams detached_params_1;

    GaitParams detached_params_2;
    GaitParams detached_params_3;

} DetachedParam;
extern DetachedParam detached_params;
extern DetachedParam state_detached_params[];

enum States {
    TROT=0,
    REALSE=2,
    STOP=1,
    START=3
};
extern enum States state;


void beziertraectory(float t, GaitParams params,float gaitoffest,float *x, float *y);
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId);
void SetCoupledPosition( int LegId,float theta1, float theta2);
void InverseKinematics(float x, float y, float *theta1, float *theta2,float leg_direction);
void gait_detached(	DetachedParam d_params,
    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);

#endif
#endif