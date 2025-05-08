#ifndef __PROJECTHEADER_H__
#define __PROJECTHEADER_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "Motor.h"
#include "pid.h"
#include "VOFA.h"
#include "VOFA_Task.h"
#include "Motor_Task.h"
#include "Chassis.h"
#include "Chassis_Task.h"
#include "Gyro_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include "Cmd_Task.h"

extern QueueHandle_t gyroYawQueue;
extern QueueHandle_t microphoneQueue;
extern QueueHandle_t cameraQueue;

typedef struct 
{
  uint8_t data[3];
} MicrophoneTypeDef;





#endif // __PROJECTHEADER_H__