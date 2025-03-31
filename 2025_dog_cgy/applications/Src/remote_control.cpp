#include "remote_control.h"



extern "C"
	
void RC_task(void *pvParameters)
{
	for(;;){
	if(rc.ch0==0&&rc.ch1==0){
		state=START;
		vTaskDelay(100);
	}
//	else if(rc.ch1>=0){
//		state=TROT;
//		vTaskDelay(100);
//	}
	else if(rc.ch0>0){
		state=ROTAT_LEFT;
		vTaskDelay(100);
	}
}
	
}