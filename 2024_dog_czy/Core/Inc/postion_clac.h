#ifndef __POSTION_CLAC__
#define __POSTION_CLAC__

#include "main.h"
#include "stdbool.h"



typedef struct {
    float stance_height;// 步行时身体与地面的期望高度 *Desired height of body from ground during walking (m)
    float step_length ;
    float up_amp; // 在贝塞尔曲线轨迹中，抬脚高(m)*Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    float freq;//一个步态周期的频率(Hz)*Frequency of one gait cycle (Hz)
    float step;	// 全步长(m)*Peak amplitude below stanceHeight in sinusoidal trajectory (m)
}GaitParams;


typedef struct {

    GaitParams detached_params_0;
    GaitParams detached_params_1;
	
    GaitParams detached_params_2;
    GaitParams detached_params_3;

} DetachedParam;



typedef struct
{
    float ref_agle[8];
    float out[8];
} temp_data;

float filter(float Data );

void gait(	DetachedParam params,
            float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
            float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void gait_detached(	GaitParams d_params,
                    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId);
void CycloidTrajectory (float t, GaitParams params, float gait_offset) ;
void BezierTrajectory (float t, GaitParams params, float gait_offset) ;
void CartesianToTheta3(float leg_direction);					
void SetCoupledPosition(int LegId);
void MoveLegs(void);
void CartesianToTheta1(float leg_direction);
void CartesianToTheta2(float leg_direction);

void SetPosition( int LegId);

bool IsValidGaitParams( GaitParams params) ;



#endif








