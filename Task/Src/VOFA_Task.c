#include "VOFA_Task.h"


TaskHandle_t VOFA_RxCallBack_TaskHandle = NULL;
TaskHandle_t VOFA_TaskHandle = NULL;

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
  VOFA_Init();
  vTaskDelay(pdMS_TO_TICKS(10));

  VOFA_RegisterData_float("targetPosX", &targetPosX);
  VOFA_RegisterData_float("targetPosY", &targetPosY);

  while (1)
  {
    VOFA_SendJustFloat(4, targetPosX, targetPosY, chassisDistance.distanceX, chassisDistance.distanceY);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}