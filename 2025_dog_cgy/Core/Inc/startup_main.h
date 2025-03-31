#ifndef __STARTUP_MAIN_H_
#define __STARTUP_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dbus.h"
//#include "string.h"


/******************define_config********************/
//�Ƿ�Ϊ�������:
//������������������0��
//�����FreeRTOS����������1��
//�����UOS����������xxxx��(��������ش���)
#define isRTOS 1

#if isRTOS == 1
#include "cmsis_os.h"
#endif


	
void startup_main(void);
void  Dog_Init();	
	

#ifdef __cplusplus
}
#endif
	
#endif
