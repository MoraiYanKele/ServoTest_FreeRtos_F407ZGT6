#ifndef __GYRO_TASK_H__
#define __GYRO_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "Motor.h"
#include "pid.h"
#include "VOFA.h"
#include "VOFA_Task.h"
#include "Motor_Task.h"
#include "Chassis.h"
#include "Chassis_Task.h"

extern TaskHandle_t Gyro_TaskHandle;
void Gyro_Task(void *argument);

extern int16_t roll;
extern int16_t pitch;
extern int16_t yaw;

#endif