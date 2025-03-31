// 由于缺少 '__config_site' 文件，通常需要检查该文件是否存在于项目中，以及其路径是否正确。
// 如果该文件是项目依赖的一部分，需要确保它已经被正确安装或配置。
// 这里假设需要添加正确的路径或包含该文件，如果 '__config_site' 是自定义头文件，可添加其路径：
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

// 控制单腿的两个电机（优化版）
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
    // 初始化电机控制器
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

    IsMotoReadyOrNot= IsReady;		//数据填充完毕

    vTaskDelay(1500);

    // state = REALSE;

    // vTaskDelay(500);

}