#ifndef __VOFA_TASK_H__
#define __VOFA_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "VOFA.h"
#include "Chassis.h"
#include "Gyro_Task.h"

extern TaskHandle_t VOFA_RxCallBack_TaskHandle;
extern TaskHandle_t VOFA_TaskHandle;

void VOFA_RxCallBack_Task(void *argument);
void VOFA_Task(void *argument);

#endif