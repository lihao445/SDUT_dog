#include "mot_move.h"
#include "CAN_PID_User.h"
#include "can.h"
#include "pid.h"
#include "tim.h"
#include "postion_clac.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "IMU_correct.h"
#include "math.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define ReductionAndAngleRatio 436.926337  //3591/187*8191/360=436.926337
#define PI 3.1415926

extern enum Gati Pid_Correspondence;
extern int min_output,max_i_output,angle_min_output,angle_i_max_output;
extern uint16_t _leg_active[];
extern float x_dog,y_dog;
extern DetachedParam Gait_Data[];
extern GaitParams state_gait_params[];
uint8_t jump_flag;
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
extern PID_Data CAN1_PID_DATA[2][4];
extern PID_Data CAN2_PID_DATA[2][4];
extern uint16_t pid_data_use;
extern enum IMU_Use IMU_state;

extern RC_ctrl_t rc_ctrl;
extern float yaw_set;
extern float yaw;
uint8_t mot_clear;
extern temp_data temp_pid;

uint8_t Pos_Star;
float ref_star_agle[8]= {0};
uint16_t pos_star_angle[8]={0};
uint16_t cl;
extern uint16_t really_star_gait;
extern uint16_t flag;
extern uint8_t _climbing_offset_flag;
extern uint8_t _climbing_offset_F;
void Tort_gait(){

								_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[0], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
//		flag=0;
							
	
}
void FTort_gait(){

								_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[20], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
//		flag=0;
							
	
}

void Walk_gait(){

	
									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
	  gait(Gait_Data[1], 0.25, 0.5, 0.0, 0.75, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
								


}

//zuo zhuan
void QLift_Zhuan(){

	
									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				IMU_state=trot;
				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[19], 0.5, 0, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0); //zuo zhuan
		MoveLegs();
								}

		

//you zhuan
void QRight_zhuan(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;

				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
  	gait(Gait_Data[18], 0.5, 0, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0); 
		MoveLegs();

								}


//zuo zhuan
void Lift_Zhuan(){

	
									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				IMU_state=trot;
				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[15], 0.5, 0, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0); //zuo zhuan
		MoveLegs();
								}



//you zhuan
void Right_zhuan(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;

				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
  	gait(Gait_Data[14], 0.5, 0, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0); 
		MoveLegs();

								}


//zuo zhuan
void Lift_gait_revolve(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
				pid_data_use=10;
		 min_output=80000,  max_i_output=80000;
		 angle_min_output=2500,angle_i_max_output=2500;
	 pid_chassis_init();
	
   gait(Gait_Data[12], 0.25, 0, 0.5, 0.75, -1.0, 1.0, 1.0, -1.0); //zuo zhuan
		MoveLegs();
		
}

//you zhuan
void Right_gait_revolve(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
				pid_data_use=10;
		 min_output=8000,  max_i_output=8000;
		 angle_min_output=2500,angle_i_max_output=2500;
	 pid_chassis_init();
	
   gait(Gait_Data[13], 0.5, 0.0, 0.5, 0.0, -1.0, 1.0, 1.0, 1.0); //you zhuan
		MoveLegs();
								
}
//you zhuan
void SRight_gait_revolve(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[21], 0.5, 0.0, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0); //you zhuan
		MoveLegs();
								
}
//zuo zhuan
void SLift_gait_revolve(){

									_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
				pid_data_use=10;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[23], 0.5, 0.0, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0); //zuo zhuan
		MoveLegs();
								
}

void Back_gait(){
_climbing_offset_flag=0;	
	_climbing_offset_F=0;
				 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[10], 0.5, 0, 0.5, 0, -1.0, -1.0, -1.0, -1.0);
		MoveLegs();
	


}
void zBack_gait(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[16], 0.5, 0, 0.5, 0, -1.0, -1.0, -1.0, -1.0);
		MoveLegs();
	


}
void yBack_gait(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
				 IMU_state=trot;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[17], 0.5, 0, 0.5, 0, -1.0, -1.0, -1.0, -1.0);
		MoveLegs();
	


}

    const float stance_height = 25.98f; 

extern float x1_dog,y1_dog,x2_dog,y2_dog;
void JUMP_PREP(){
	_climbing_offset_flag=0;
_climbing_offset_F=0;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
	     x_dog = 2;
			y_dog =20;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	     x_dog = 2;
			y_dog = 20;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}
void JUMP_PREP2(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
	     x_dog = 6;
			y_dog =20;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	     x_dog = 6;
			y_dog = 18;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}

void JUMP_PREP3(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
	     x_dog = 12;
			y_dog =22;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	     x_dog = 12;
			y_dog = 18;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}

void JUMP_Star(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=16000,max_i_output=16000;
		 angle_min_output=6000,angle_i_max_output=6000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
//	     x_dog = -stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(6*PI/180);
	x_dog = 3;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
//	     x_dog = stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(10*PI/180);
	x_dog = 3;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}

void JUMP_Star2(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=16000,max_i_output=16000;
		 angle_min_output=6000,angle_i_max_output=6000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
//	     x_dog = -stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(6*PI/180);
	x_dog = 9;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
//	     x_dog = stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(10*PI/180);
	x_dog = 9;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}
void JUMP_Star3(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=16000,max_i_output=16000;
		 angle_min_output=6000,angle_i_max_output=6000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
//	     x_dog = -stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(6*PI/180);
	x_dog = 12;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
//	     x_dog = stance_height*sin(10*PI/180);
//			y_dog = stance_height*cos(10*PI/180);
	x_dog = 12;
	y_dog = 40;
	CartesianToTheta3(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}




void Xie_Po(){
    _climbing_offset_flag=1;
	_climbing_offset_F=0;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[7], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();

return;

					}
void Xie_Po_down(){	
	_climbing_offset_flag=0;
	_climbing_offset_F=1;
			 pid_data_use=10;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[22], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();

return;

					}

void Xie_Po_Zuo(){

						    _climbing_offset_flag=1;
						_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[8], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
return;

}

void Xie_Po_You(){

    _climbing_offset_flag=1;
							_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[9], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
return;

}
void Xie_Po_Zuo_down(){

						    _climbing_offset_flag=1;
						_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[5], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
return;

}

void Xie_Po_You_down(){

    _climbing_offset_flag=1;
							_climbing_offset_F=0;
			 pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
  	gait(Gait_Data[6], 0.5, 0, 0.5, 0, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
return;

}



void Stop_gait(){
_climbing_offset_flag=0;
	_climbing_offset_F=0;
		  	pid_data_use=10;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=3000,angle_i_max_output=3000;
				pid_chassis_init();
	
//	CAN1_CMD_1(pid_call_1(-0*437,1),
//			pid_call_1(0*437,2),
//			pid_call_1(-0*437,3),
//			pid_call_1(0*437,4));

//		CAN2_CMD_1(pid_call_2(0,1),
//			pid_call_2(0,2),
//			pid_call_2(0,3),
//			pid_call_2(0,4));
	x_dog=0;
	y_dog=25.98;
	
		    CartesianToTheta3(1.0);
			for(uint8_t m=0;m<4;m++){
			SetPosition(m);
			}
	
					MoveLegs();
}

void Tort_gait_qiaoqiao(){
					if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=3000,max_i_output=3000;
		 angle_min_output=800,angle_i_max_output=800;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[9], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
						if(really_star_gait==1){

			 IMU_state=trot;
			 pid_data_use=0;
		   min_output=12000,max_i_output=12000;
		 angle_min_output=2000,angle_i_max_output=2000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[9], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
						}

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim8){   
				  	pid_data_use=10;
		   min_output=9000,max_i_output=9000;
		 angle_min_output=3500,angle_i_max_output=3500;
				pid_chassis_init();
		cl++;
			 		CAN1_CMD_1(pid_call_1(119.0f*437,1),
			pid_call_1(-19.8f*437,2),
			pid_call_1(15.1f*437,3),
			pid_call_1(-116.5f*437,4));
//			CAN1_CMD_1(pid_call_1(-47.0f*437,1),
//			pid_call_1(-50.0f*437,2), 
//			pid_call_1(48.0f*437,3),
//			pid_call_1(51.0f*437,4));
				

				CAN2_CMD_1(pid_call_2(-23.5f*437,1),
			pid_call_2(125.9f*437,2),
			pid_call_2(-129.4f*437,3),
			pid_call_2(30.55f*437,4));
//				CAN2_CMD_1(pid_call_2(-46.0f*437,1),
//			pid_call_2(-47.0f*437,2),
//			pid_call_2(50.0f*437,3),
//			pid_call_2(48.0f*437,4));		



		
 if(cl==2000){

					memset(&motor_can1,0,sizeof(motor_can1));
	 	     memset(&motor_can1,0,sizeof(motor_measure_t)*4);
	 	     memset(&motor_can2,0,sizeof(motor_measure_t)*4);
    for(int i=0; i<8; i++)
   temp_pid.ref_agle[i]=ref_star_agle[i]=0;
 	 HAL_TIM_Base_Stop_IT(&htim8);
	 HAL_TIM_Base_Stop(&htim8);  //关闭定时器
 

	}
}
	/*******************************************************************************************************/
	
	if(htim==&htim1){
		//zuo 1   you  0
		if(cl==2000){
			

				if(rc_ctrl.rc.s[0]==3&&rc_ctrl.rc.s[1]==3){
				//往前行走
			if(rc_ctrl.rc.ch[3]>500){
			 FTort_gait();
				return;
			}
			

				//Zuo_Zhuan			


			if(rc_ctrl.rc.ch[0]>500){
			SRight_gait_revolve();
			return;
			}
			
			if(rc_ctrl.rc.ch[0]<-500){
			SLift_gait_revolve();
				return;
			}                             	
			

			if(rc_ctrl.rc.ch[1]==660){
						 JUMP_Star();
//							jump_flag=0;
							return;
			}
			
			if(rc_ctrl.rc.ch[1]==-660){
			JUMP_Star2();
			     return;
			}
		
//					if(rc_ctrl.rc.ch[1]<-200){
//						Tort_gait_qiaoqiao();
//						return;
//						}

						if(rc_ctrl.rc.ch[2]>600){
						QRight_zhuan();
						return;
						}
						if(rc_ctrl.rc.ch[2]==-660){
						QLift_Zhuan();
						return;
						}
						
						
			if(rc_ctrl.rc.ch[4]==-660){
							Lift_gait_revolve();
							yaw_set=yaw;

				return;
			}
			  //You_Zhuan
			if(rc_ctrl.rc.ch[4]==660){
//							Right_gait_Xuan_Zhuan();
//								yaw_set=yaw;
							 JUMP_PREP();
				return;
			}
						if(rc_ctrl.rc.ch[3]==-660){
			JUMP_PREP2();
			return;
			}
			flag=0;
			really_star_gait=0;
					Stop_gait();

			jump_flag=0;
			_leg_active[0]=0,_leg_active[1]=0,_leg_active[2]=0,_leg_active[3]=0;
							return;
		}
		
		
		
		
		
		/********hua  li  de  fen  ge xian********/
		
				if(rc_ctrl.rc.s[0]==1&&rc_ctrl.rc.s[1]==3){
					
			if(rc_ctrl.rc.ch[3]>500){
			Tort_gait();	
				return;
			}
				//Zuo_Zhuan			
			if(rc_ctrl.rc.ch[2]==-660){
							Lift_Zhuan();
				return;
			}
			  //You_Zhuan
			if(rc_ctrl.rc.ch[2]>500){
							Right_zhuan();
							
				return;
				
				
				
			}
						
			if(rc_ctrl.rc.ch[3]<-500){
						Back_gait();
				return;
				}
			
				//Xie  Po
			if(rc_ctrl.rc.ch[1]==660){
							Xie_Po();
							return;
			}
							//Xie  Po
			if(rc_ctrl.rc.ch[0]<-600){
							zBack_gait();
							return;
			}
						if(rc_ctrl.rc.ch[0]>600){
							yBack_gait();
							return;
			}
						if(rc_ctrl.rc.ch[1]==-660){
						Right_gait_revolve();
						return;
						}
			flag=0;
			really_star_gait=0;
			Stop_gait();
			jump_flag=0;
			_leg_active[0]=0,_leg_active[1]=0,_leg_active[2]=0,_leg_active[3]=0;

							return;
						
				}
				
				
						/********               斜坡专用              ********/
				if(rc_ctrl.rc.s[0]==3&&rc_ctrl.rc.s[1]==1){
				//往前行走
			if(rc_ctrl.rc.ch[3]>500){
			 Xie_Po();
				return;
			}
			

				//Zuo_Zhuan			


			if(rc_ctrl.rc.ch[0]>500){
			Xie_Po_You_down();
			return;
			}
			
			if(rc_ctrl.rc.ch[0]<-500){
			Xie_Po_down();
				return;
			}                             	
			

			if(rc_ctrl.rc.ch[1]==660){
						 JUMP_Star3();
//							jump_flag=0;
							return;
			}
			
			if(rc_ctrl.rc.ch[1]==-660){
			JUMP_Star2();
			     return;
			}
		
//					if(rc_ctrl.rc.ch[1]<-200){
//						Tort_gait_qiaoqiao();
//						return;
//						}

						if(rc_ctrl.rc.ch[2]>600){
						Xie_Po_Zuo();
						return;
						}
						if(rc_ctrl.rc.ch[2]==-660){
						Xie_Po_You();
						return;
						}
						
						
			if(rc_ctrl.rc.ch[4]==-660){
							Lift_gait_revolve();
							yaw_set=yaw;

				return;
			}
			  //You_Zhuan
			if(rc_ctrl.rc.ch[4]==660){
//							Right_gait_Xuan_Zhuan();
//								yaw_set=yaw;
							 JUMP_PREP2();
				return;
			}
						if(rc_ctrl.rc.ch[3]==-660){
			JUMP_PREP3();
			return;
			}
			flag=0;
			really_star_gait=0;
					Stop_gait();

			jump_flag=0;
			_leg_active[0]=0,_leg_active[1]=0,_leg_active[2]=0,_leg_active[3]=0;
							return;
		}
		

		}
		}
	
	}


	
	
	
