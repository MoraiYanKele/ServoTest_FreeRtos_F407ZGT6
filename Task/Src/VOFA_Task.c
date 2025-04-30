#include "VOFA_Task.h"


TaskHandle_t VOFA_RxCallBack_TaskHandle = NULL;
TaskHandle_t VOFA_TaskHandle = NULL;
extern QueueHandle_t vofaQueue; // 声明队列句柄
extern ChassisTypeDef chassis; // 声明底盘结构体

void VOFA_RxCallBack_Task(void *argument)
{
  /* Infinite loop */
  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      VOFA_RxCallBack();
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void VOFA_Task(void *argument)
{
  VOFA_SendFireWater("ready\n"); // 发送准备就绪指令
  // VOFA_RegisterData_float("targetPosX", &chassis.targetPosX);
  // VOFA_RegisterData_float("targetPosY", &chassis.targetPosY);
  // // VOFA_RegisterData_float("chassisSpeedW", &chassis.chassisSpeed->vW);
  // VOFA_RegisterData_float("kp", &chassis.positionPidX->Kp);
  // VOFA_RegisterData_float("ki", &chassis.positionPidX->Ki);
  // VOFA_RegisterData_float("kd", &chassis.positionPidX->Kd);

  while (1)
  {
    // VOFA_SendJustFloat(14,       
    //   // chassis.chassisSpeed->vX,
    //   // chassis.chassisSpeed->vY,
    //   // chassis.chassisSpeed->vW,
    //   chassis.positionPidX->Kp,
    //   chassis.positionPidX->Ki,
    //   chassis.targetPosX,
    //   chassis.targetPosY,
    //   chassis.motorA->speed,
    //   chassis.motorB->speed,
    //   chassis.motorC->speed,
    //   chassis.motorD->speed,
    //   chassis.motorA->position,
    //   chassis.motorB->position,
    //   chassis.motorC->position,
    //   chassis.motorD->position,
    //   chassis.chassisDistance->distanceX,
    //   chassis.chassisDistance->distanceY);
    // // VOFAQueueTypeDef vofaQueueReceive;
    // // if (xQueueReceive(vofaQueue, &vofaQueueReceive, portMAX_DELAY) == pdTRUE)
    // // {
    // //   VOFA_SendJustFloat_Queue(vofaQueueReceive.data, vofaQueueReceive.dataLength); // 发送数据
    // // }
  }
}

void VOFA_SendDataToVOFA(float *data, uint16_t length)
{
  if (length > 0 && length <= VOFA_MAX_CHANNELS)
  {
    VOFAQueueTypeDef vofaQueueSend;
    vofaQueueSend.dataLength = length;
    memcpy(vofaQueueSend.data, data, sizeof(float) * length);
    xQueueSend(vofaQueue, &vofaQueueSend, portMAX_DELAY); // 非阻塞发送数据到队列
  }
  else
  {
    // 错误处理：长度超出范围
  }
  
}