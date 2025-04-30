#include "CallBack.h"


extern TaskHandle_t VOFA_TaskHandle;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart == VOFA_UART) 
  {
    TxCallBack_DoubleBufferUartDMA(&uartToVOFA);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (huart == VOFA_UART)
  {
    vTaskNotifyGiveFromISR(VOFA_RxCallBack_TaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  if (huart == CMD_2_4G_UART)
  {
    VOFA_SendFireWater("receive\n"); // 发送接收成功指令
    xTaskNotifyFromISR(Cmd_TaskHandle, NOTIFY_FROM_CMD_2_4G_UART_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  if (huart == CMD_CAMERA_UART)
  {
    xTaskNotifyFromISR(Cmd_TaskHandle, NOTIFY_FROM_CMD_CAMERA_UART_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
}
