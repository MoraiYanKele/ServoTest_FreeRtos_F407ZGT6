#include "Cmd_Task.h"
#include <limits.h>

uint8_t rxBufferFrom2_4G[64] = {0};
uint8_t rxBufferFromCamera[64] = {0};
uint8_t txBufferFor2_4G[7] = {0};

extern ChassisModeTypeDef chassisMode; // 0x01
extern uint8_t qrNumberInCameraProcessing;

void CMD_2_4G_CallBack();
uint8_t CMD_Camera_CallBack();


TaskHandle_t Cmd_TaskHandle = NULL;


#define DEBUG_EN  0

void Cmd_Task(void *argument)
{
  HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, sizeof(rxBufferFrom2_4G)); // 启动2.4GDMA接收
  // HAL_UARTEx_ReceiveToIdle_DMA(CMD_CAMERA_UART, rxBufferFromCamera, sizeof(rxBufferFromCamera)); // 启动摄像头DMA接收
  while (1)
  {
    uint32_t ulNotifiedValue; // 用于存储接收到的通知值
    BaseType_t xResult = xTaskNotifyWait(0x00, // 清除通知值
                                         ULONG_MAX, // 清除所有通知值
                                         &ulNotifiedValue, // 存储接收到的通知值
                                        portMAX_DELAY); // 等待通知
    
    // uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, 64);
    #if DEBUG_EN
      VOFA_SendFireWater("%02X %02X %02X %02X %02X %02X %02X\n", rxBufferFrom2_4G[0], rxBufferFrom2_4G[1], rxBufferFrom2_4G[2], rxBufferFrom2_4G[3], rxBufferFrom2_4G[4], rxBufferFrom2_4G[5], rxBufferFrom2_4G[6]);
      
    #else 
    if (xResult == pdPASS)
    {
      if (ulNotifiedValue & NOTIFY_FROM_CMD_2_4G_UART_ISR) 
      {
        CMD_2_4G_CallBack();
      }

      else if (ulNotifiedValue & NOTIFY_FROM_CMD_CAMERA_UART_ISR)
      {
        if (CMD_Camera_CallBack() != 0)
        {
          qrNumberInCameraProcessing = CMD_Camera_CallBack(); // 处理摄像头数据
        } 
      }

      else if (ulNotifiedValue & NOTIFY_FROM_CAMERA_PROCESSING)
      {
       HAL_UARTEx_ReceiveToIdle_DMA(CMD_CAMERA_UART, rxBufferFromCamera, sizeof(rxBufferFromCamera)); // 启动摄像头DMA接收
      }
    }
    #endif 
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, sizeof(rxBufferFrom2_4G)); // 启动2.4GDMA接收
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void CMD_2_4G_CallBack()
{
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
        else if (rxBufferFrom2_4G[2] == 0x02 && rxBufferFrom2_4G[3] == 0x02 && rxBufferFrom2_4G[4] == 0x02 && rxBufferFrom2_4G[5] == 0x02)
        {
          chassisMode = CHASSIS_TASK2_READY;
        }
        break;
      }
      case 0x04:
      {
        // 错误处理
      }
      default:
        break;
    }
  }
}

uint8_t CMD_Camera_CallBack()
{
  if (rxBufferFromCamera[0] == 'a')
  {
    uint8_t CameraData[3] = {0};
    
    for (int i = 0; i < 3; i++)
    {
      CameraData[i] = rxBufferFromCamera[i + 1] - '0'; // 读取数据
    }
    uint8_t qrNumber = (CameraData[0] * 100) + (CameraData[1] * 10) + CameraData[2];
    if (qrNumber > 0 && qrNumber <= 12)
    {
      return qrNumber; // 返回二维码数据
    }
    else 
    {
      return 0;
    }
  }
  else
  {
    return 0; // 返回0表示没有二维码数据
  }
}