#ifndef __CMD_TASK_H__
#define __CMD_TASK_H__

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
#include "ProjectHeader.h"

#define CMD_2_4G_UART           &huart3

#define PACKET_SIZE 7

extern TaskHandle_t CmdFrom2_4G_TaskHandle;
void CmdFrom2_4G_Task(void *argument);

#endif