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
#include "queue.h" // Include FreeRTOS queue definitions
#include "VOFA.h"
#include "Motor.h"
#include "VOFA_Task.h"
#include "Motor_Task.h"
#include "Chassis_Task.h"
#include "Gyro_Task.h"
#include "VOFAQUeueType.h"
#include "ProjectHeader.h"
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
TaskHandle_t Init_TaskHandle = NULL;

QueueHandle_t vofaQueue = NULL; // 声明队列句柄
QueueHandle_t gyroYawQueue = NULL; // 声明队列句柄
QueueHandle_t microphoneQueue = NULL; // 声明队列句柄
QueueHandle_t cameraQueue = NULL; // 声明队列句柄

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Init_Task(void *argument)
{
  HAL_TIM_Base_Start_IT(&htim5);
  VOFA_Init();

  vTaskDelete(NULL);  
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  vofaQueue = xQueueCreate(10, sizeof(VOFAQueueTypeDef));
  gyroYawQueue = xQueueCreate(1, sizeof(float));
  microphoneQueue = xQueueCreate(3, sizeof(MicrophoneTypeDef)); // 创建麦克风队列
  cameraQueue = xQueueCreate(3, sizeof(uint8_t)); // 创建摄像头队列 
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(Init_Task, "Init_Task", 128, NULL, osPriorityRealtime, &Init_TaskHandle);
  xTaskCreate(VOFA_RxCallBack_Task, "VOFA_RxCallBack_Task", 256, NULL, osPriorityNormal1, &VOFA_RxCallBack_TaskHandle);
  xTaskCreate(Moto1_Task, "Moto1_Task", 128, NULL, osPriorityHigh, &Motor1_TaskHandle);
  xTaskCreate(Moto2_Task, "Moto2_Task", 128, NULL, osPriorityHigh, &Motor2_TaskHandle);
  xTaskCreate(Moto3_Task, "Moto3_Task", 128, NULL, osPriorityHigh, &Motor3_TaskHandle);
  xTaskCreate(Moto4_Task, "Moto4_Task", 128, NULL, osPriorityHigh, &Motor4_TaskHandle);
  xTaskCreate(Chassis_Task, "Chassis_Task", 512, NULL, osPriorityNormal, &Chassis_TaskHandle);
  xTaskCreate(VOFA_Task, "VOFA_Task", 256, NULL, osPriorityNormal, &VOFA_TaskHandle);
  xTaskCreate(Cmd_Task, "Cmd_Task", 256, NULL, osPriorityNormal, &Cmd_TaskHandle);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
 
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

