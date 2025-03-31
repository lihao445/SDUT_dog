#include "pid_user.h"
#include "can_receive.h"

/*PID����������*/
PID_Controller pid_controller;
/***************/

//PID�������(�ṹ��)
//pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_yaw;
pid_type_def pid_pos_x;
pid_type_def pid_pos_y;
pid_type_def pid_LF_v[2], pid_RF_v[2], pid_LB_v[2], pid_RB_v[2];
pid_type_def pid_LF_p[2], pid_RF_p[2], pid_LB_p[2], pid_RB_p[2];

//���PID����
// fp32 motor_LF[3]={20,0,0.5};
// fp32 motor_RF[3]={20,0,0.5};
// fp32 motor_LB[3]={20,0,0.5};    
// fp32 motor_RB[3]={20,0,0.5};
fp32 motor_V[3]={11,0.0, 0.05};
fp32 motor_pos[3]={1.05,0.0, 0.1};

//��λPID����
fp32 motor_yaw_pid[3] = {12,0,0.1};
fp32 motor_pos_x_pid[3] = {6,0,0};
fp32 motor_pos_y_pid[3] = {6,0,0};

/**
 * @brief       PID�豸��ʼ��
 * @param       void
 * @retval      void
 * @note        ���ｫ���е�PID�豸�Ĳ������г�ʼ��������Kp,Ki,Kd,I_limit(�����޷�),O_limit(���޷�)���������,����ֵ������pid_type_def����С�
 */ 

void PID_Controller::All_Device_Init(void)
{
	for(int i=0;i<2;i++){
	this->core.PID_Init(&pid_LF_v[i], PID_POSITION, motor_V,10000, 6000);
	this->core.PID_Init(&pid_RF_v[i], PID_POSITION, motor_V,10000, 6000);
	this->core.PID_Init(&pid_LB_v[i], PID_POSITION, motor_V,10000, 6000);
	this->core.PID_Init(&pid_RB_v[i], PID_POSITION, motor_V,10000, 6000);

	this->core.PID_Init(&pid_LF_p[i], PID_POSITION, motor_pos,4000, 3000);
	this->core.PID_Init(&pid_RF_p[i], PID_POSITION, motor_pos,4000, 3000);
	this->core.PID_Init(&pid_LB_p[i], PID_POSITION, motor_pos,4000, 3000);
	this->core.PID_Init(&pid_RB_p[i], PID_POSITION, motor_pos,4000, 3000);
	// this->core.PID_Init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
	// this->core.PID_Init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 4000, 3000);
	}
	
  //��λPID
	this->core.PID_Init(&pid_yaw,PID_POSITION,motor_yaw_pid,3000,1500);
	this->core.PID_Init(&pid_pos_x,PID_POSITION,motor_pos_x_pid,1500,1500);
	this->core.PID_Init(&pid_pos_y,PID_POSITION,motor_pos_y_pid,1500,1500);
}

/**
 * @brief       CAN1�ٶȻ�
 * @param       set_speed���ٶ�rpm
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */


fp32 PID_Controller::CAN_MOTOR::CAN1_Velocity_Realize_LF(fp32 set_speed,int i)
{
	
	pid_controller.core.PID_Calc(&pid_LF_v[i],can_bus.motor_LF[i].speed_rpm , set_speed);
	return pid_LF_v[i].out;
}

fp32 PID_Controller::CAN_MOTOR::CAN1_Velocity_Realize_RF(fp32 set_speed,int i)
{
	
	pid_controller.core.PID_Calc(&pid_RF_v[i],can_bus.motor_RF[i].speed_rpm , set_speed);
	return pid_RF_v[i].out;
}


/**
 * @brief       CAN1λ�û�
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */

fp32 PID_Controller::CAN_MOTOR::CAN1_Position_Realize_LF(fp32 set_pos,int i)
{


	pid_controller.core.PID_Calc(&pid_LF_p[i],can_bus.motor_LF[i].total_angle , set_pos);
	return pid_LF_p[i].out;
}

fp32 PID_Controller::CAN_MOTOR::CAN1_Position_Realize_RF(fp32 set_pos,int i)
{
	pid_controller.core.PID_Calc(&pid_RF_p[i],can_bus.motor_RF[i].total_angle , set_pos);
	return pid_RF_p[i].out;
}

/**
 * @brief       CAN1�����ٶ�˫��
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */

fp32 PID_Controller::CAN_MOTOR::CAN1_VP_Dual_Loop_Realize_LF(fp32 set_pos,int i)
{
	return CAN1_Velocity_Realize_LF(CAN1_Position_Realize_LF(set_pos,i),i);
}
fp32 PID_Controller::CAN_MOTOR::CAN1_VP_Dual_Loop_Realize_RF(fp32 set_pos,int i)
{
	return CAN1_Velocity_Realize_RF(CAN1_Position_Realize_RF(set_pos,i),i);
}


/**
 * @brief       CAN2�ٶȻ�
 * @param       set_speed���ٶ�rpm
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_Velocity_Realize_LB(fp32 set_speed,int i)
{
	
	pid_controller.core.PID_Calc(&pid_LB_v[i],can_bus.motor_LB[i].speed_rpm , set_speed);
	return pid_LB_v[i].out;
}
fp32 PID_Controller::CAN_MOTOR::CAN2_Velocity_Realize_RB(fp32 set_speed,int i)
{
	
	pid_controller.core.PID_Calc(&pid_RB_v[i],can_bus.motor_RB[i].speed_rpm , set_speed);
	return pid_RB_v[i].out;
}

/**
 * @brief       CAN2λ�û�
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_Position_Realize_LB(fp32 set_pos,int i)
{
	pid_controller.core.PID_Calc(&pid_LB_p[i],can_bus.motor_LB[i].total_angle , set_pos);
	return pid_LB_p[i].out;
}
fp32 PID_Controller::CAN_MOTOR::CAN2_Position_Realize_RB(fp32 set_pos,int i)
{
	pid_controller.core.PID_Calc(&pid_RB_p[i],can_bus.motor_RB[i].total_angle , set_pos);
	return pid_RB_p[i].out;
}


/**
 * @brief       CAN2�����ٶ�˫��
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_VP_Dual_Loop_Realize_LB(fp32 set_pos,int i)
{
	return CAN2_Velocity_Realize_LB(CAN2_Position_Realize_LB(set_pos,i),i);
}
fp32 PID_Controller::CAN_MOTOR::CAN2_VP_Dual_Loop_Realize_RB(fp32 set_pos,int i)
{
	return CAN2_Velocity_Realize_RB(CAN2_Position_Realize_RB(set_pos,i),i);
}

/**
 * @brief       �����PID
 * @param       set_yaw��Ŀ�꺽���
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Yaw_Realize(fp32 set_yaw)
{
	//PID_calc(&pid_yaw,absolute_chassis_measure.Euler.yaw_total,set_yaw);
	//return pid_yaw.out;
	(void)set_yaw;
	return 0;
}

/**
 * @brief       X����PID
 * @param       set_pos_x��Ŀ��X����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */

fp32 PID_Controller::SENSORS::Pos_X_Realize(fp32 set_pos_x)
{
	//PID_calc(&pid_pos_x,absolute_chassis_measure.Position.Pos_X,set_pos_x);
	//return pid_pos_x.out;
	(void)set_pos_x;
	return 0;
}

/**
 * @brief       Y����PID
 * @param       set_pos_y��Ŀ��Y����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Pos_Y_Realize(fp32 set_pos_y)
{
	//PID_calc(&pid_pos_y,absolute_chassis_measure.Position.Pos_Y,set_pos_y);
	//return pid_pos_y.out;
	(void)set_pos_y;
	return 0;
}

