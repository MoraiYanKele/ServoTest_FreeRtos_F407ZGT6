#include "VOFA_Task.h"


TaskHandle_t VOFA_RxCallBack_TaskHandle = NULL;
TaskHandle_t VOFA_TaskHandle = NULL;
extern QueueHandle_t vofaQueue; // 声明队列句柄


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
  while (1)
  {
    VOFAQueueTypeDef vofaQueueReceive;
    if (xQueueReceive(vofaQueue, &vofaQueueReceive, portMAX_DELAY) == pdTRUE)
    {
      VOFA_SendJustFloat_Queue(vofaQueueReceive.data, vofaQueueReceive.dataLength); // 发送数据
    }
  }
}

void VOFA_SendDataToVOFA(float *data, uint16_t length)
{
  if (length > 0 && length <= VOFA_MAX_CHANNELS)
  {
    VOFAQueueTypeDef vofaQueueSend;
    vofaQueueSend.dataLength = length;
    memcpy(vofaQueueSend.data, data, sizeof(float) * length);
    xQueueSend(vofaQueue, &vofaQueueSend, 0); // 非阻塞发送数据到队列
  }
  else
  {
    // 错误处理：长度超出范围
  }
  
}