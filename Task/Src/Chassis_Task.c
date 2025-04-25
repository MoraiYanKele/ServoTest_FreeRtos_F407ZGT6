#include "Chassis_Task.h"

TaskHandle_t Chassis_TaskHandle = NULL;

PIDControllerTypedef positionPidForX;
PIDControllerTypedef positionPidForY;

ChassisTypeDef chassis = 
{
  .motorA = &motor1,
  .motorB = &motor2,
  .motorC = &motor3,
  .motorD = &motor4,
  
  .chassisSpeed = &chassisSpeed,
  .wheelSpeed = &wheelSpeed,

  .positionPidX = &positionPidForX,
  .positionPidY = &positionPidForY
};

float targetPosX = 0;
float targetPosY = 0;
float targetPosW = 0;

void TIM5CallBack_Task() // 100Hz, 10ms
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  Chassis_GetEncoder(&chassis);
  
  vTaskNotifyGiveFromISR(Motor1_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor2_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor3_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor4_TaskHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
} 


void Chassis_Task(void *argument)
{
  Chassis_Init();
  PID_Init(&positionPidForX, 1.5, 0.0, 0.0, 0.01, 100, 100, 0.0f, PID_MODE_POSITION);
  PID_Init(&positionPidForY, 1.5, 0.0, 0.0, 0.01, 100, 100, 0.0f, PID_MODE_POSITION);

  if (yaw != 0)
  {
    targetPosW = (float)yaw;
  }
  else
  {
    targetPosW = 0;
  }

  while (1)
  {
    targetSpeed1 = -PIDCompute(&positionPidForX, -position1, targetPosX);
    targetSpeed2 = -PIDCompute(&positionPidForY, -position2, targetPosY);
    targetSpeed3 = PIDCompute(&positionPidForX, position3, targetPosX);
    targetSpeed4 = PIDCompute(&positionPidForY, position4, targetPosY);


    GetDistance(&chassisDistance);


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}