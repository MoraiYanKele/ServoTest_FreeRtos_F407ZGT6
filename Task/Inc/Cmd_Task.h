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

#define NOTIFY_FROM_CMD_2_4G_UART_ISR     (1UL << 0) // 第 0 位代表 UART3 Idle
#define NOTIFY_FROM_CMD_CAMERA_UART_ISR   (1UL << 1) // 第 1 位代表 UART5 Idle
#define NOTIFY_FROM_CAMERA_PROCESSING     (1UL << 2) // 第 2 位代表 摄像头进程

#define CMD_2_4G_UART           &huart3
#define CMD_CAMERA_UART         &huart5

#define PACKET_SIZE 7

extern TaskHandle_t Cmd_TaskHandle;
void Cmd_Task(void *argument);
#endif