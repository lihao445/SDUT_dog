#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#ifdef __cplusplus
#endif

#pragma once

#define M_PI 3.1415926f
// 硬件参数
#define GEAR_RATIO         19.0f    // 减速比
#define ENCODER_RES        8192     // 编码器分辨率（脉冲/圈）

// 轨迹参数
#define STEP_LENGTH        100.0f    // 步长 (mm) 
#define LIFT_HEIGHT        30.0f    // 抬腿高度 (mm)
#define CYCLE_TIME        	1.0f     // 步态周期 (秒)
#define BODY_HEIGHT       -259.8f  // 初始机身高度 (mm)

// 机械参数
#define L1                 150.0f   // 大腿长度 (mm)
#define L2                 300.0f   // 小腿长度 (mm)

// 查表法参数
#define LUT_SIZE           200      // 轨迹点数量


void InitTrajectoryLUT();
void GetTrajectoryPoint(float t, float *x, float *y);
//void InverseKinematics(float x, float y, float *theta1, float *theta2);


#ifdef __cplusplus
#endif

#endif
