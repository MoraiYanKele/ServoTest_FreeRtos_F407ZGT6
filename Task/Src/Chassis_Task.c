#include "Chassis_Task.h"

TaskHandle_t Chassis_TaskHandle = NULL;
extern ChassisTypeDef chassis;

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
  Chassis_Init(&chassis);
  PID_Init(chassis.positionPidX, 1.5, 0.0, 0.0, 0.01, 100, 100, 0.0f, PID_MODE_POSITION);
  PID_Init(chassis.positionPidX, 1.5, 0.0, 0.0, 0.01, 100, 100, 0.0f, PID_MODE_POSITION);

  while (1)
  {
    SetChassisSpeed(&chassis);
    GetDistance(&chassis);


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}