#ifndef __VOFA_TASK_H__
#define __VOFA_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "VOFA.h"
#include "Chassis.h"
#include "Gyro_Task.h"
#include "VOFAQUeueType.h"

#include "ProjectHeader.h"


extern TaskHandle_t VOFA_RxCallBack_TaskHandle;
extern TaskHandle_t VOFA_TaskHandle;

extern QueueHandle_t vofaQueue; // 声明队列句柄

void VOFA_RxCallBack_Task(void *argument);
void VOFA_Task(void *argument);
void VOFA_SendDataToVOFA(float *data, uint16_t length);

#endif