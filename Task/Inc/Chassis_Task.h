#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

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
#include "Gyro_Task.h"

extern TaskHandle_t Chassis_TaskHandle;

extern PIDControllerTypedef positionPidForX;
extern PIDControllerTypedef positionPidForY;


extern float targetPosX;
extern float targetPosY;
extern float targetPosW;

void TIM5CallBack_Task(); // 100Hz, 10ms
void Chassis_Task(void *argument);

#endif