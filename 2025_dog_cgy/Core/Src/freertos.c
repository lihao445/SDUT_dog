/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "task_user.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId WALKHandle;
osThreadId PostureControlHandle;
osThreadId MOTOR_CTRLHandle;
osThreadId Remote_controlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void walk_m(void const * argument);
void PostureControl_task(void const * argument);
void MotorControl_task(void const * argument);
void RC_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of WALK */
//  osThreadDef(WALK, walk_m, osPriorityNormal, 0, 128);
//  WALKHandle = osThreadCreate(osThread(WALK), NULL);

  /* definition and creation of PostureControl */
  osThreadDef(PostureControl, PostureControl_task, osPriorityAboveNormal, 0, 256);
  PostureControlHandle = osThreadCreate(osThread(PostureControl), NULL);

  /* definition and creation of MOTOR_CTRL */
  osThreadDef(MOTOR_CTRL, MotorControl_task, osPriorityNormal, 0, 128);
  MOTOR_CTRLHandle = osThreadCreate(osThread(MOTOR_CTRL), NULL);

  /* definition and creation of Remote_control */
  osThreadDef(Remote_control, RC_task, osPriorityHigh, 0, 128);
  Remote_controlHandle = osThreadCreate(osThread(Remote_control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_walk_m */
/**
  * @brief  Function implementing the WALK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_walk_m */
__weak void walk_m(void const * argument)
{
  /* USER CODE BEGIN walk_m */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END walk_m */
}

/* USER CODE BEGIN Header_PostureControl_task */
/**
* @brief Function implementing the TROT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PostureControl_task */
__weak void PostureControl_task(void const * argument)
{
  /* USER CODE BEGIN PostureControl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PostureControl_task */
}

/* USER CODE BEGIN Header_MotorControl_task */
/**
* @brief Function implementing the MOTOR_CTRL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControl_task */
__weak void MotorControl_task(void const * argument)
{
  /* USER CODE BEGIN MotorControl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorControl_task */
}

/* USER CODE BEGIN Header_RC_task */
/**
* @brief Function implementing the Remote_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_task */
__weak void RC_task(void const * argument)
{
  /* USER CODE BEGIN RC_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RC_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
