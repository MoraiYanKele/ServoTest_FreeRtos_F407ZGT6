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
#include "Gyro_Task.h"

#include "ProjectHeader.h"


extern TaskHandle_t Chassis_TaskHandle;

typedef enum
{
  CHASSIS_READY = 0x01,
  CHASSIS_TASK1_READY = 0x02,
  CHASSIS_TASK2_READY = 0x03,

  CHASSIS_TASK1_GO_THROUGH = 0x04,
  CHASSIS_TASK1_COMPLETE = 0x05,

  CHASSIS_TASK2_GO_THROUGH = 0x06,
  CHASSIS_TASK2_COMPLETE = 0x07
} ChassisModeTypeDef;

typedef enum 
{
  TASK1_MOVING_X          = 0x01,     // 正在移动到当前目标点的 X 坐标
  TASK1_MOVING_Y          = 0x02,     // 正在移动到当前目标点的 Y 坐标
  TASK1_REACH_PROCESSING  = 0x03,     // 到达任务点后处理
  TASK1_WAITING_QR_RESULT = 0x04,     // 等待二维码结果
  TASK1_COMPLETE          = 0x05      // 所有点都已到达
} Task1GoThroughTypeDef;

typedef enum
{
  TASK2_NOTIFYING_CAMERA          = 0x01, // 摄像头处理二维码通知
  TASK2_WAITING_QR_RESULT         = 0x02, // 等待二维码结果
  TASK2_NOTIFYING_2_4G            = 0x03, // 
  TASK2_MOVING_X                  = 0x04,     // 正在移动到当前目标点的 X 坐标
  TASK2_MOVING_Y                  = 0x05,     // 正在移动到当前目标点的 Y 坐标
  TASK2_COMPLETE                  = 0x06,     // 所有点都已到达  
} Task2GoThroughTypeDef;

typedef struct {
  float x;
  float y;
  uint8_t isQRPoint;    // 是否是关键点(有二维码) 0:没有二维码 1:有二维码 2:起始点 3:终点
  uint8_t qrNumber;     // 二维码内容
  char *qrArea;        // 二维码区域
  uint16_t ServoDegree; // 舵机角度
} PointTypeDef;

void TIM5CallBack_Task(); // 100Hz, 10ms
void Chassis_Task(void *argument);
void SendReceiveSuccess();

// uint8_t IsINRange(float nowValue, float targetValue, float range);
// uint8_t MoveToTargetAndCheck(float targetX, float targetY, float range);
// void Task1GoThrough();
// void SendQrNumber(PointTypeDef *point);

#endif