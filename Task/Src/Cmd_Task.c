#include "Cmd_Task.h"


uint8_t rxBufferFrom2_4G[64] = {0};
uint8_t txBufferFor2_4G[7] = {0};

extern ChassisModeTypeDef chassisMode; // 0x01




TaskHandle_t CmdFrom2_4G_TaskHandle = NULL;


#define DEBUG_EN  0

void CmdFrom2_4G_Task(void *argument)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBufferFrom2_4G, 64); // 启动DMA接收

  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBufferFrom2_4G, 64);
    #if DEBUG_EN
      VOFA_SendFireWater("%02X %02X %02X %02X %02X %02X %02X\n", rxBufferFrom2_4G[0], rxBufferFrom2_4G[1], rxBufferFrom2_4G[2], rxBufferFrom2_4G[3], rxBufferFrom2_4G[4], rxBufferFrom2_4G[5], rxBufferFrom2_4G[6]);
      
    #else 
    
    if (rxBufferFrom2_4G[0] == 0xAA && rxBufferFrom2_4G[6] == 0x0F)
    {
      switch (rxBufferFrom2_4G[1])
      {
      case 0x01: // 
        {
          if (rxBufferFrom2_4G[2] == 0x01 && rxBufferFrom2_4G[3] == 0x01 && rxBufferFrom2_4G[4] == 0x01 && rxBufferFrom2_4G[5] == 0x01)
          {
            chassisMode = CHASSIS_TASK1_READY;
            
          }
          break;
        }
      
      default:
        break;
      }
    }

    #endif 
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
