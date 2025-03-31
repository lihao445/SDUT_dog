// ����ȱ�� '__config_site' �ļ���ͨ����Ҫ�����ļ��Ƿ��������Ŀ�У��Լ���·���Ƿ���ȷ��
// ������ļ�����Ŀ������һ���֣���Ҫȷ�����Ѿ�����ȷ��װ�����á�
// ���������Ҫ�����ȷ��·����������ļ������ '__config_site' ���Զ���ͷ�ļ����������·����
#include "bsp_motor.h"
#include <math.h>

fp32 ref_the1[4] = {0};
fp32 ref_the2[4] = {0};
temp_data temp={0};
bool IsMotoReadyOrNot= NotReady;

extern "C"	
void MotorControl_task(void *pvParameters)
{
    for(;;) {
      ControlLegMotors();
    }

}

// ���Ƶ��ȵ�����������Ż��棩
void ControlLegMotors() {
    if(IsMotoReadyOrNot== IsReady) {
        for(int i=0; i<4; i++){
            ref_the1[i]=temp.ref_theta1[i];
            ref_the2[i]=temp.ref_theta2[i];
        }
        IsMotoReadyOrNot= NotReady;
    }
		can_bus.cmd.CAN1_Front(
			pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_LF( ref_the1[0], 0), 
            pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_LF(-ref_the2[0], 1),
			pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_RF( ref_the2[1], 0), 
			pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_RF(-ref_the1[1], 1)
		);
		
		can_bus.cmd.CAN2_Front(
			pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_RB(-ref_the2[2], 0),
			pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_RB(ref_the1[2], 1),
			pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_LB(-ref_the1[3], 0), 
			pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_LB( ref_the2[3], 1)
        );
    osDelay(3);
} 

void motor_init() {
    // ��ʼ�����������
    // #define MOTOR_STOP(motor) motor(0, 0), motor(0, 1)
    // can_bus.cmd.CAN1_Front(
    //     MOTOR_STOP(pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_LF),
    //     MOTOR_STOP(pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_RF)
    // );
    // can_bus.cmd.CAN2_Front(
    //     MOTOR_STOP(pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_RB),
    //     MOTOR_STOP(pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_LB)
    // );
    // #undef MOTOR_STOP

    temp.ref_theta1[0] = 0;
    temp.ref_theta2[0] = 0;
    temp.ref_theta1[1] = 0;
    temp.ref_theta2[1] = 0;
    temp.ref_theta1[2] = 0;
    temp.ref_theta2[2] = 0;
    temp.ref_theta1[3] = 0;
    temp.ref_theta2[3] = 0;

    IsMotoReadyOrNot= IsReady;		//����������

    vTaskDelay(1500);

    // state = REALSE;

    // vTaskDelay(500);

}