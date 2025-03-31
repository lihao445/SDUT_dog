#include "startup_main.h"
#include "bsp_delay.h"
#include "can_receive.h"
#include "pid_user.h"
//#include "tim.h"

void EnableFPU(void) {  
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  
    __DSB();  
    __ISB();  
}

void startup_main(void)
{
	
	//�δ�ʱ����ʼ��
  bsp_delay.f4.Init(168);
	
//PID��ʼ��
	pid_controller.All_Device_Init();
	
//CANͨ���˲���ʼ��
	can_bus.bsp.Filter_Init(&hcan1);
	can_bus.bsp.Filter_Init(&hcan2);
	
//CANͨ�ſ���
 	can_bus.bsp.CAN_Start(&hcan1);
	can_bus.bsp.CAN_Start(&hcan2);

//������ʱ���ж�
	//HAL_TIM_Base_Start_IT(&htim7);
//������DT7&DR16ң����ͨ��	
	dbus_uart_init();

	EnableFPU();

	//Dog_Init();
	
	//vTaskStartScheduler();	

#if isRTOS==0    	//������������Ϊ0��rtos����Ϊ1
	for(;;)  //��ͬ��while(true)
	{
		//Dog_Init();

	}
#endif
}

void Dog_Init()
{
	can_bus.cmd.CAN1_Front(
		pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_LF(0, 0), 
		pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_LF(0, 1),
		pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_RF(0, 0), 
		pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize_RF(0, 1)
	);
	can_bus.cmd.CAN2_Front(
		pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_RB(0, 0), 
		pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_RB(0, 1), 
		pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_LB(0, 0), 
		pid_controller.can_motor.CAN2_VP_Dual_Loop_Realize_LB(0, 1)
	);
}
