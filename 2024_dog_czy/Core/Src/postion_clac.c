#include "postion_clac.h"
#include "can.h"
#include "pid.h"
#include "math.h"
#include "main.h"
#include "CAN_PID_User.h"
#include "stdbool.h"

#define L1 15
#define L2 30

#define arefa 0.001

/*  腿部平行时  13fd3421.856f  */
#define PI 3.1415926
float x_dog=0,y_dog=0,theta2,theta1;
temp_data temp_pid= {0};
float now_time;
#define ReductionAndAngleRatio 436.926337f
               //157293/360 电机转子转过一定角度所需脉冲数
#define true 1


float gp;
uint16_t _leg_active[]={0,0,0,0};
extern uint16_t Time_count;
float _climbing_offset_angle=18;
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
 uint16_t really_star_gait;
 uint16_t flag;
 
//static float oldOutData = 0.0f;
//double nowData,get_Data,nowOutData;
// 
//float filter(float Data )
//{
//	
//    nowData  = Data; 
//    nowOutData = arefa * nowData  + (1.0f - arefa) * oldOutData;
//    oldOutData = nowOutData;
//    return nowOutData;  
//}



GaitParams state_gait_params[]={
	    //{身高, 步长, 抬腿高, 频率}	单位 cm
				{25.98,  0.0  ,6.0,   3},
				{16.0, 12.00, 5.0,  3.0},//walk
				{17.0, 0.00, 6.00,  2.0}, 	//BOUND
				{27.3, 12.0 ,20.0 , 1.0},
				{17.3,    12.0,4.0,   1},					//原地转

};
//typedef struct {
//    float stance_height;// 步行时身体与地面的期望高度 *Desired height of body from ground during walking (m)
//    float step_length ;// 全步长(cm)*Peak amplitude below stanceHeight in sinusoidal trajectory (m)
//    float up_amp; // 在正弦轨迹中，足部峰值在支架高度以上(m)*Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
//    float freq;//一个步态周期的频率(Hz)*Frequency of one gait cycle (Hz)
//}GaitParams;

DetachedParam Gait_Data[]={
		  //{身高, 无用, 抬腿高, 频率，步长}	单位 cm
{
				{25.98,  10.0,3,    2.5,2.5},
				{25.98,  10.0,3,    2.5,2.5},
				{25.98,  10.0 ,3,    2.5,2.2},
				{25.58,  10.0 ,3,    2.5,2.5},
},//trot1  0

{
				{26.0, 1.0, 4.0,      1.5,0.1},
			  {26.0, 1.0, 4.0,      1.5,0.1},	
				{26.0, 1.0, 4.0,      1.5,0.1},
				{26.0, 1.0, 4.0,      1.5,0.1},
},			//walk   1

{
				{16.0, 6.0, 2.5,  1.6,3.0},
				{16.0, 2.0, 2.5,  1.6,3.0},
				{16.0, 2.0, 2.5,  1.6,3.0},
				{16.0, 6.0, 2.5,  1.6,3.0},
},//Zuo_Zhuan   2

{
				{16.0, 2.0, 2.5,  1.8,3.0},
				{16.0, 6.0, 2.5,  1.8,3.0},
				{16.0, 6.0, 2.5,  1.8,3.0},
				{16.0, 2.0, 2.5,  1.8,3.0},
},//You_Zhuan  3   hao xiang bi zhixing zhi

{
				{16,  4.0  ,2.5,        1.8,3.0},
				{16,  4.0  ,2.5,         1.8,3.0},
				{16,  4.0 , 2.5,         1.8,3.0},
				{16,  4.0 , 2.5,         1.8,3.0},
},	//duo jiao   4
	

{
				{18,  1.6  ,1.0,         2.0,1.0},
				{18,  1.6  ,1.0,         2.0,1.0},
				{20,  1.6 , 3.0,         2.0,3.0},
				{20,  1.6 , 3.0,         2.0,3.5},
},//Xie Po Zuo Xia Qu 5

{
				{18.0,  6.8 , 1.0,         2.0,1.0},
				{18.0,  6.8,  1.0,         2.0,1.0},
				{20.0,  6.8 , 3.0,         2.0,3.5},
				{20.0,  6.8 , 3.0,         2.0,3.0},
},//Xie Po You Xia Qu 6

{
				{20,  5.0  ,3,         2.0,3.25},
				{20,  5.0  ,3,         2.0,3.0},
				{18,  1.0  , 1,         2.0,1.0},
				{18,  1.0  , 1,         2.0,1.0},
},//Xie Po Ce Shi  7



{
				{20.0, 2.0, 3.0,     2.0,3.5},
				{20.0, 2.0, 3.0, 2.0,3.0},
				{18.0, 2.0, 1.0,  2.0,1.0},
				{18.0, 2.0, 1.0,  2.0,1.0},
},//Xie Po Zuo Shang Qu  8

{
				{20,  2.0  ,3.0,         2.0,3.0},
				{20,  2.0  ,3.0,        2.0,3.5},
				{18,  2.0 , 1.0,         2.0,1.0},
				{18,  2.0 , 1.0,         2.0,1.0},
},//Xie Po You Shang Qu   9

{
				{23.98,  4.0  ,1,        3,3.0},
				{23.98,  4.0  ,1,        3,2.0},
				{23.98,  4.0 , 1,       3,3.0},
				{23.98,  4.0 , 1,       3,3.0},
},//back   10

{
				{16,  1.8  ,2.5,       2.2,3.0},
				{16,  1.8  ,2.5,         2.2,3.0},
				{16,  1.8 , 2.5,     2.2,3.0},
				{16,  1.8, 2.5,       2.2,3.0},
},   //11

{
				{25.98,  1  ,6,         2,3.0},
				{25.98,  1  ,6,         2,3.0},
				{25.98,  1 , 6,         2,3.0},
				{25.98,  1 , 6,         2,3.0},
},//zuo xuan zhuan 12

{
				{25.98,  1.0  ,1,         2,1.0},
				{25.98,  1.0  ,4,        3,8.0},
				{25.98,  1.0 , 1,        2,1.0},
				{25.98,  1.0 , 4,       3,8.0},
},//you xuan zhuan 13

{
				{24.98,  1.0  ,6,         3.0,1.0},
				{24.98,  4.0 ,6,     3.0,10.0},
				{24.98,  4.0  ,6,      3.0,8.0},
				{24.98,  1.0 , 6,        3.0,1.0},
},//  14
    
{
				{24.98,  6.0  ,6,        3.0,10.0},
				{24.98,  1.0  ,6,       3.0,1.0},
				{24.98,  1.0 , 6,      3.0,1.0},
				{24.98,  6.0 , 6,       3.0,8.0},
},//  15

{
				{23.98,  1.0  ,3,         1.5,1.0},
				{24.98,  6.0 ,3,     1.5,4.0},
				{24.98,  6.0  ,3,      1.5,4.0},
				{24.98,  1.0 , 3,        1.5,1.0},
},//  16

{
				{23.98,  6.0  ,3,        1.5,4.0},
				{23.98,  1.0  ,2,       1.5,4},
				{23.98,  1.0 , 2,      1.5,4},
				{23.98,  6.0 , 3,       1.5,4.0},
},//  17

{
				{24.98,  1.0  ,2,         1.5,1.0},
				{24.98,  4.0 ,2,     3.0,8.0},
				{24.98,  4.0  ,2,      3.0,8.0},
				{24.98,  1.0 , 2,        1.5,1.0},
},//  18
    
{
				{24.98,  6.0  ,2,        3.0,8.0},
				{24.98,  1.0  ,2,       1.5,1.0},
				{24.98,  1.0 , 2,      1.5,1.0},
				{24.98,  6.0 , 2,       3.0,8.0},
},//  19

{
				{23,  10.0,0.5,   3,4},
				{22,  10.0,0.5,    3,4},
				{22,  10.0 ,0.5,    3,4},
				{23,  10.0 ,0.5,    3,4},
},//Ftrot1  20

{
				{23,  1.0  ,2,         2,4.0},
				{23,  1.0  ,2,        2,3.0},
				{23,  1.0 , 2,        2,4.0},
				{23,  1.0 , 2,       2,4.0},
},//you xuan zhuan 21

{
				{18,  1.0  ,1,         2,1.0},
				{18,  1.0  ,1,        2,1.0},
				{20,  1.0 , 3,        2,3.0},
				{20,  1.0 , 3,       2,3.0},
},//xie_po_down 22

{
				{23,  1.0  ,2,         2,4.0},
				{23,  1.0  ,2,        2,4.0},
				{23,  1.0 , 2,        2,4.0},
				{23,  1.0 , 2,       2,4.0},
},//zou xuan zhuan 23
};

void gait_detached( GaitParams d_params,
                     float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                     float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction) {

    float t = HAL_GetTick()/1000.0f-now_time/1000.0f;
								
    // const float leg0_direction = 1.0;
											 
			//CAN1的12电机
    if(_leg_active[0]==1)
        CoupledMoveLeg( t, d_params, leg0_offset, leg0_direction, 0);

    // const float leg1_direction = 1.0;
		
		//CAN1的34电机
    if(_leg_active[1]==1)
        CoupledMoveLeg( t, d_params, leg1_offset, leg1_direction, 1);

    // const float leg2_direction = 1.0;
		
		//CAN2的12电机
    if(_leg_active[2]==1)
        CoupledMoveLeg( t, d_params, leg2_offset, leg2_direction, 2);

    //  const float leg3_direction = 1.0;
		//CAN2的34电机
    if(_leg_active[3]==1)
        CoupledMoveLeg( t, d_params, leg3_offset, leg3_direction, 3);
		
}


//  	gait(Gait_Data[0], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
void gait(	DetachedParam params,
            float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
            float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction) {


    float t = HAL_GetTick()/1000.0f-now_time/1000.0f;
		
    //printf("\r\n t=%f",t);

    // const float leg0_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_0, leg0_offset, leg0_direction, 0);

    // const float leg1_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_1, leg1_offset, leg1_direction, 1);

    // const float leg2_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_2, leg2_offset, leg2_direction, 2);

    //  const float leg3_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_3, leg3_offset, leg3_direction, 3);

    //????PD
    // ChangeTheGainsOfPD(gains);
}

void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId)
{		

    BezierTrajectory(t, params, gait_offset);		
    CartesianToTheta3(leg_direction);		
    SetCoupledPosition(LegId);		
	
}

float x1_dog,y1_dog;
float x2_dog,y2_dog;
void BezierTrajectory(float t, GaitParams params, float gaitOffset) {
    static float p = 0;
    static float prev_t = 0;

    float stanceHeight = params.stance_height;

    float upAMP = params.up_amp;

    float stepLength = params.step;
    float FREQ = params.freq;

	  p += FREQ*(t - prev_t);
    prev_t = t;

    float gp =fmod((p +gaitOffset), 1.0);
	
	

			float t_normalized = 2.0f * gp ;

    if (gp <=1.0f/4.0f) {	
        // 摆动阶段的贝塞尔曲线系数

			
        x_dog =stepLength / 2 - stepLength * (10*pow(t_normalized, 3)*(1-t_normalized)*(1-t_normalized) + 5*pow(t_normalized, 4)*(1-t_normalized) + pow(t_normalized, 5));
			  y_dog =  stanceHeight - upAMP* (10*pow(2.0f*t_normalized, 3)*(1-2.0f*t_normalized)*(1-2.0f*t_normalized) + 5*pow(2.0f*t_normalized, 4)*(1-2.0f*t_normalized) + pow(2.0f*t_normalized, 5));
//		        x_dog=0;
//				y_dog=0;
		}
			if(gp >= 1.0f/4.0f && gp <= 1.0f/2.0f)	
			{
			 x_dog = stepLength / 2 - stepLength * (10*pow(t_normalized, 3)*(1-t_normalized)*(1-t_normalized) + 5*pow(t_normalized, 4)*(1-t_normalized) + pow(t_normalized, 5));
			  y_dog =  stanceHeight - upAMP * (10*pow(2-2.0f*t_normalized, 3)*pow(2.0f*t_normalized-1, 2) + 5*pow(2-2.0f*t_normalized, 4)*(2.0f*t_normalized-1) + pow(2-2.0f*t_normalized, 5));
//        x_dog=0;
//				y_dog=0;
		} 
        // 支撑阶段的贝塞尔曲线系数
     if(gp >= 1.0f/2.0f && gp < 1.0f){

        x_dog =stepLength / 2 - stepLength * (10*pow(2-t_normalized, 3)*pow(t_normalized-1, 2) + 5*pow(2-t_normalized, 4)*(t_normalized-1) + pow(2-t_normalized, 5));
        //y_dog = stanceHeight + downAMP * (10*pow(1-t_normalized, 3)*t_normalized*t_normalized*t_normalized + 5*pow(1-t_normalized, 4)*t_normalized*t_normalized + 2*pow(1-t_normalized, 5));
//		 x_dog=0;
//		 y_dog=0;
				y_dog =stanceHeight*2.15f/2.0f;
		 }
  

}	
//void BezierTrajectory(float t, GaitParams params, float gaitOffset) {
//    static float p = 0;
//    static float prev_t = 0;

//    float stanceHeight = params.stance_height;
//    float stepLength = params.step;
//    float upAMP = params.up_amp;
//    float FREQ = params.freq;

//	  p += (t - prev_t);
//    prev_t = t;

//    float gp =FREQ*fmod((p +gaitOffset), 1.0);
//	
//	

//			float t_normalized = 2.0f * gp/FREQ ;

//    if (gp <=FREQ/4.0f) {	
//        // 摆动阶段的贝塞尔曲线系数

//			
//        x_dog = stepLength / 2.0f - stepLength * (10*pow(t_normalized, 3)*(1-t_normalized)*(1-t_normalized) + 5*pow(t_normalized, 4)*(1-t_normalized) + pow(t_normalized, 5));
//			  y_dog =  stanceHeight - upAMP* (10*pow(2.0f*t_normalized, 3)*(1-2.0f*t_normalized)*(1-2.0f*t_normalized) + 5*pow(2.0f*t_normalized, 4)*(1-2.0f*t_normalized) + pow(2.0f*t_normalized, 5));
////		        x_dog=0;
////				y_dog=0;
//		}
//			if(gp >= FREQ/4.0f && gp <= FREQ/2.0f)	
//			{
//			 x_dog = stepLength / 2.0f - stepLength * (10*pow(t_normalized, 3)*(1-t_normalized)*(1-t_normalized) + 5*pow(t_normalized, 4)*(1-t_normalized) + pow(t_normalized, 5));
//			  y_dog =  stanceHeight - upAMP * (10*pow(2-2.0f*t_normalized, 3)*pow(2.0f*t_normalized-1, 2) + 5*pow(2-2.0f*t_normalized, 4)*(2.0f*t_normalized-1) + pow(2-2.0f*t_normalized, 5));
////        x_dog=0;
////				y_dog=0;
//		} 
//        // 支撑阶段的贝塞尔曲线系数
//     if(gp >= FREQ/2.0f && gp < FREQ){

//        x_dog =stepLength / 2.0f - stepLength * (10*pow(2-t_normalized, 3)*pow(t_normalized-1, 2) + 5*pow(2-t_normalized, 4)*(t_normalized-1) + pow(2-t_normalized, 5));
//        //y_dog = stanceHeight + downAMP * (10*pow(1-t_normalized, 3)*t_normalized*t_normalized*t_normalized + 5*pow(1-t_normalized, 4)*t_normalized*t_normalized + 2*pow(1-t_normalized, 5));
////		 x_dog=0;
////		 y_dog=0;
//				y_dog =stanceHeight*2.25f/2.0f;
//		 }
//    

//}	




void CartesianToTheta3(float leg_direction)
{
	    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;

    L=sqrt(		pow(x_dog,2)	+		pow(y_dog,2)	);

    if(L<5.0f) L=15.0f;
    else if(L>42.0f) L=42.0f;

    // vTaskSuspend(MotorControlTask_Handler);

    N=asin(x_dog/L)*180.0/PI;
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;
    A1=M-N;

    A2=M+N;

    if(leg_direction==1.0f) {
        theta2=(A1-90.0f);
        theta1=(A2-90.0f);
    } else if(leg_direction==-1.0f) {
        theta1=(A1-90.0f);
        theta2=(A2-90.0f);
    }


}

uint8_t _climbing_offset_flag;
uint8_t _climbing_offset_F;

void SetCoupledPosition( int LegId)
{
	//CAN1的12电机
	if(_climbing_offset_flag==1){
        if(LegId==0)
        {
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
        }
	//CAN1的34电机

        else if(LegId==1)
        {
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
        }
	//CAN2的12电机

        else if(LegId==2)
        {
            temp_pid.ref_agle[4]=-theta1*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[5]=-theta2*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
        }
		//CAN2的34电机

        else if(LegId==3)
        {
            temp_pid.ref_agle[7]=theta1*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[6]=theta2*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
        }
			}
	else if(_climbing_offset_F==1){
		if(LegId==0)
        {
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
        }
	//CAN1的34电机

        else if(LegId==1)
        {
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
        }
	//CAN2的12电机

        else if(LegId==2)
        {
            temp_pid.ref_agle[4]=-theta1*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[5]=-theta2*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
        }
		//CAN2的34电机

        else if(LegId==3)
        {
            temp_pid.ref_agle[7]=theta1*ReductionAndAngleRatio-_climbing_offset_angle*ReductionAndAngleRatio;
            temp_pid.ref_agle[6]=theta2*ReductionAndAngleRatio+_climbing_offset_angle*ReductionAndAngleRatio;
        }
			}
				
    else {
        if(LegId==0)
        {
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio;
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio;
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio;
        }

        else if(LegId==2)
        {
            temp_pid.ref_agle[4]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[5]=-theta2*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            temp_pid.ref_agle[7]=theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[6]=theta2*ReductionAndAngleRatio;
        }
    	
}
		
}

void SetPosition( int LegId){
//1shun 2ni 3shun 4ni
 if(LegId==0)
        {
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio;
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio;
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio;
        }

        else if(LegId==2)
        {
            temp_pid.ref_agle[4]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[5]=-theta2*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            temp_pid.ref_agle[7]=theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[6]=theta2*ReductionAndAngleRatio;
        }



}




float pos_agle[8];


 
 bool IsValidGaitParams( GaitParams params) {
    const float maxL = 42.0f;
    const float minL = 15.0f;

    float stanceHeight = params.stance_height;
    float upAMP = params.up_amp;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    if (stanceHeight  > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0f, 2)) > maxL) {
        return false;
    }
    if (stanceHeight - upAMP < minL) {

        return false;
    }



    if (FREQ < 0) {
 
        return false;
    }

    if (FREQ > 10.0f) {
;
        return false;
    }

    return true;
}
 
 void MoveLegs()
{
 for(uint8_t i=0; i<8; i++){
 pos_agle[i]=temp_pid.ref_agle[i];
 }
 
		CAN1_CMD_1(pid_call_1(pos_agle[0],1),
			pid_call_1(pos_agle[1],2),
			pid_call_1(pos_agle[2],3),
			pid_call_1(pos_agle[3],4));

		CAN2_CMD_1(pid_call_2(pos_agle[4],1),
			pid_call_2(pos_agle[5],2),
			pid_call_2(pos_agle[6],3),
			pid_call_2(pos_agle[7],4));




 }
 

