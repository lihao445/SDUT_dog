#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#ifdef __cplusplus
#endif

#pragma once

#define M_PI 3.1415926f
// Ӳ������
#define GEAR_RATIO         19.0f    // ���ٱ�
#define ENCODER_RES        8192     // �������ֱ��ʣ�����/Ȧ��

// �켣����
#define STEP_LENGTH        100.0f    // ���� (mm) 
#define LIFT_HEIGHT        30.0f    // ̧�ȸ߶� (mm)
#define CYCLE_TIME        	1.0f     // ��̬���� (��)
#define BODY_HEIGHT       -259.8f  // ��ʼ����߶� (mm)

// ��е����
#define L1                 150.0f   // ���ȳ��� (mm)
#define L2                 300.0f   // С�ȳ��� (mm)

// �������
#define LUT_SIZE           200      // �켣������


void InitTrajectoryLUT();
void GetTrajectoryPoint(float t, float *x, float *y);
//void InverseKinematics(float x, float y, float *theta1, float *theta2);


#ifdef __cplusplus
#endif

#endif
