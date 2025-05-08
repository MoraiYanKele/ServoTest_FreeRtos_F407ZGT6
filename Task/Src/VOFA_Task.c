#include "VOFA_Task.h"


TaskHandle_t VOFA_RxCallBack_TaskHandle = NULL;
TaskHandle_t VOFA_TaskHandle = NULL;
extern QueueHandle_t vofaQueue; // 声明队列句柄
extern ChassisTypeDef chassis; // 声明底盘结构体
extern uint8_t qrNumberInTask1; // 声明二维码处理标志

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
  VOFA_RegisterData_float("targetPosW", &chassis.targetPosW); // 发送目标位置
  VOFA_RegisterData_float("softStartFactor", &chassis.softStartFactor);// 发送软启动系数
  VOFA_RegisterData_float("kp", &chassis.positionPidW->Kp); // 发送PID参数
  VOFA_RegisterData_float("ki", &chassis.positionPidW->Ki); // 发送PID参数
  VOFA_RegisterData_float("kd", &chassis.positionPidW->Kd); // 发送PID参数


  while (1)
  {
    // VOFA_SendJustFloat(6,
    //   chassis.chassisDistance->distanceX,
    //   chassis.chassisSpeed->vX,
    //   chassis.motorA->speed,
    //   chassis.motorB->speed,
    //   chassis.motorC->speed,d
    //   chassis.motorD->speed
    // );
    // VOFA_SendJustFloat(4, chassis.yaw, chassis.targetPosW, chassis.chassisSpeed->vW, chassis.wheelSpeed->vA); // 发送当前角度和目标角度

    // VOFA_SendFireWater("yaw: %f, %f\n", chassis.yaw, chassis.targetPosW);
    vTaskDelay(pdMS_TO_TICKS(10)); //



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