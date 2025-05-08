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

extern ChassisModeTypeDef chassisMode; // 0x01
extern uint8_t qrAreaInTask2[2]; // 二维码区域
extern uint8_t qrNumberInTask1;
extern uint8_t qrNumberInTask2; 
extern ChassisTypeDef chassis; // 底盘结构体

#define NOTIFY_FROM_CMD_2_4G_UART_ISR           (1UL << 0) // 第 0 位代表 UART3 Idle
#define NOTIFY_FROM_CMD_CAMERA_UART_ISR         (1UL << 1) // 第 1 位代表 UART5 Idle
#define NOTIFY_FROM_CAMERA_PROCESSING           (1UL << 2) // 第 2 位代表 摄像头进程
#define NOTIFY_FROM_TASK2_TO_GET_QR_NUMBER      (1UL << 3) // 第 3 位代表 任务2的通知
#define NOTIFY_FROM_TASK2_TO_GET_QR_AREA        (1UL << 4) // 第 4 位代表 任务2的通知
#define NOTIFY_FROM_TASK1_MICROPHONE            (1UL << 5) // 第 5 位代表 任务2的通知

#define NOTIFY_FROM_GYRO_UART_ISR         (1UL << 3) // 第 3 位代表 摄像头进程结束

#define NOTIFY_QR_RESULT_READY            (1UL << 0)
#define NOTIFY_QR_AREA_GOTTEN             (1UL << 1) // 第 1 位代表 摄像头进程结束

#define CMD_2_4G_UART           &huart2
#define CMD_CAMERA_UART         &huart3
#define CMD_GYRO_UART           &huart5
#define CMD_MICROPHONE_UART     &huart4

#define PACKET_SIZE 7

extern TaskHandle_t Cmd_TaskHandle;
void Cmd_Task(void *argument);
#endif