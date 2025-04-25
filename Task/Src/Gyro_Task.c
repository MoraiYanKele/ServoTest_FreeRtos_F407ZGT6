#include "Gyro_Task.h"


TaskHandle_t Gyro_TaskHandle = NULL;


uint8_t receiveData[128];

int16_t roll = 0;
int16_t pitch = 0;
int16_t yaw = 0;

void Gyro_Task(void *argument)
{

  HAL_UART_Receive_DMA(&huart5, receiveData, 11);
  while (1)
  {
    
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      
      roll = (receiveData[3] << 8) | receiveData[2];
      pitch = (receiveData[5] << 8) | receiveData[4];
      yaw = (receiveData[7] << 8) | receiveData[6];
      HAL_UART_Receive_DMA(&huart5, receiveData, 11);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));

  }
}