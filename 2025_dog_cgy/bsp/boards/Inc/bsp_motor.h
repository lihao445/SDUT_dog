#ifndef __BSP_MOTOR_H_
#define __BSP_MOTOR_H_

#include "struct_typedef.h"
#ifdef __cplusplus
extern "C" 
{
#endif

#include "startup_main.h"
#include "can_receive.h"
#include "pid_user.h"
#include "trajectory.h"	
#include "dbus.h"


#ifdef __cplusplus
}
#define IsReady  1
#define NotReady 0

#include "posture_ctrl.h"

typedef struct
{
    // float ref_LF[2];
    // float ref_RF[2];
    // float ref_LB[2];
    // float ref_RB[2];
    fp32 ref_theta1[4];
    fp32 ref_theta2[4];
    float out[8];
} temp_data;
extern temp_data temp;
extern bool IsMotoReadyOrNot;

void ControlLegMotors();
void motor_init();

#endif

#endif


