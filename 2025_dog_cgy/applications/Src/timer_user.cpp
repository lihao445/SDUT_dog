#include "timer_user.h"
#include "can_receive.h"
#include "pid_user.h"
#include "startup_main.h"


//extern "C"
/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       htim：触发中断的定时器句柄
 * @retval      void
 * @note        该函数由中断公共服务函数调用，不用用户去调用。且为一个弱函数，所以在C++中要在该函数前面加上extern "C"，或直接用extern "C"{}括起来
 */              
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM7)  //周期为1ms
//	{
//        //Dog_Init();                 
//	}
//}


