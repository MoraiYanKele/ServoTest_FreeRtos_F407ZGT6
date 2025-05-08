#include "Chassis_Task.h"


extern ChassisTypeDef chassis;

// void GetYaw(ChassisTypeDef* chassis);
uint8_t IsINRange(float nowValue, float targetValue, float range);
void Task1GoThrough();
void Task2GoThrough();
void SendQrNumber(PointTypeDef *point);
// void SendMessageToMicrophone(uint8_t message);
void SendQrNumberToGetArea(uint8_t qrNumber);

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
#define IS_DEBUG 0// 是否开启调试模式


uint8_t qrNumberInTask1 = 0;
uint8_t qrNumberInTask2 = 0; 
uint8_t qrAreaInTask2[2] = {0};
uint8_t isStart = 0; // 是否开始任务

ChassisModeTypeDef chassisMode = CHASSIS_READY; // 0x01
static Task1GoThroughTypeDef goThroughMode = TASK1_MOVING_X; // 任务1的状态
Task2GoThroughTypeDef goThroughMode2  = TASK2_NOTIFYING_CAMERA;

TaskHandle_t Chassis_TaskHandle = NULL;
void Chassis_Task(void *argument)
{
  const float softStartIncrement = 0.01f; // 软启动系数
  const float softStartMax = 1.0f; // 最大软启动系数

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // 启动PWM
  TIM9->CCR1 = 190;

  Chassis_Init(&chassis);
  PID_Init(chassis.positionPidX, 1.5, 0.0, 0.0, 0.01, 300, 100, 5.0f, PID_MODE_POSITION);
  PID_Init(chassis.positionPidY, 1.5, 0.0, 0.0, 0.01, 300, 100, 5.0f, PID_MODE_POSITION);
  PID_Init(chassis.positionPidW, 9.0, 0.0, 0.0, 0.01, 150, 100, 0.25f, PID_MODE_POSITION);

  while (1)
  {
    #if IS_DEBUG
    static float lastTargetPosX = 0.0f;
    static float lastTargetPosY = 0.0f;
    GetDistance_FourWheel(&chassis);

    // if (chassis.softStartFactor < softStartMax)
    // {
    //   if (chassis.softStartFactor < 0.5)
    //   {
    //     chassis.softStartFactor += softStartIncrement;
    //   }
    //   else
    //   {
    //     chassis.softStartFactor += softStartIncrement * 2; // 软启动系数增加速度减半
    //   }
    //   if (chassis.softStartFactor >= softStartMax)
    //   {
    //     chassis.softStartFactor = softStartMax;
    //   }
    // } // 软启动系数逐渐增加)
    // if (IsINRange(chassis.chassisDistance->distanceX, chassis.targetPosX, 5.0f))
    // {
    //   lastTargetPosX = chassis.targetPosX;
    //   lastTargetPosY = chassis.targetPosY;
    //   chassis.softStartFactor = 0.0f; // 到达目标点后，软启动系数归零
    // }
    
    chassis.chassisSpeed->vX = PIDCompute(chassis.positionPidX, chassis.chassisDistance->distanceX, chassis.targetPosX);
    chassis.chassisSpeed->vY = PIDCompute(chassis.positionPidY, chassis.chassisDistance->distanceY, chassis.targetPosY);
    chassis.chassisSpeed->vW = PIDCompute(chassis.positionPidW, chassis.yaw, chassis.targetPosW);



    OmniWheelKinematics_FourWheel(&chassis);
    SetChassisSpeed_WithoutPID(&chassis);

    #else
    if (chassis.softStartFactor < softStartMax)
    {
      if (chassis.softStartFactor < 0.5)
      {
        chassis.softStartFactor += softStartIncrement;
      }
      else
      {
        chassis.softStartFactor += softStartIncrement * 2;
      }
    }
    else if (chassis.softStartFactor >= softStartMax)
    {
      chassis.softStartFactor = softStartMax;
    }

    // GetYaw(&chassis); // 获取陀螺仪数据
    GetDistance_FourWheel(&chassis);

    chassis.chassisSpeed->vX = PIDCompute(chassis.positionPidX, chassis.chassisDistance->distanceX, chassis.targetPosX);
    chassis.chassisSpeed->vY = PIDCompute(chassis.positionPidY, chassis.chassisDistance->distanceY, chassis.targetPosY);
    chassis.chassisSpeed->vW = PIDCompute(chassis.positionPidW, chassis.yaw, chassis.targetPosW);
    OmniWheelKinematics_FourWheel(&chassis);
    SetChassisSpeed_WithoutPID(&chassis);

    switch (chassisMode)
    {
      case CHASSIS_READY:
      vTaskDelay(pdMS_TO_TICKS(1000));

        break;
      case CHASSIS_TASK1_READY:
        // 处理任务1的逻辑
        SendReceiveSuccess();
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (isStart == 0 && chassis.yaw != 0.0f)
        {
          chassis.targetPosW = chassis.yaw;
          isStart = 1;
        }

 
        goThroughMode = TASK1_MOVING_X;
        chassisMode = CHASSIS_TASK1_GO_THROUGH;
        break;
     
      case CHASSIS_TASK1_GO_THROUGH:
        {
          
          Task1GoThrough();
        }
        break;
      case CHASSIS_TASK1_COMPLETE:
      {
        chassis.motorA->targetSpeed = 0;
        chassis.motorB->targetSpeed = 0;
        chassis.motorC->targetSpeed = 0;
        chassis.motorD->targetSpeed = 0;
        break; 
      }
      case CHASSIS_TASK2_READY:
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (isStart == 0 && chassis.yaw != 0.0f)
        {
          chassis.targetPosW = chassis.yaw;
          isStart = 1;
        }
        goThroughMode2 = TASK2_NOTIFYING_CAMERA;
        chassisMode = CHASSIS_TASK2_GO_THROUGH;

        break;

      case CHASSIS_TASK2_GO_THROUGH:
        {
          // 处理任务2的逻辑
          Task2GoThrough();
        }
        break;
      case CHASSIS_TASK2_COMPLETE:


        chassisMode = CHASSIS_READY; // 返回到准备状态

        break;
      default:
      
        break;
    }
    #endif
    


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static PointTypeDef targetPoints[] = 
{
  {0, 300, 1, 0, "A1", 190},   // Point 1
  {0, 600, 2, 0, "A2", 190},   // Point 2
  {0, 900, 3, 0, "A3", 190},   // Point 3
  {0, 1350, 0, 0, NULL, 190},  // Point 4
  {-800, 1350, 0, 0, NULL, 190}, // Point 5
  {-800, 900, 4, 0, "B3", 465},  // Point 6
  {-800, 900, 5, 0, "C3", 190},  // Point 7
  {-800, 600, 6, 0, "C2", 190},  // Point 8
  {-800, 600, 7, 0, "B2", 465},  // Point 9
  {-800, 300, 8, 0, "B1", 465},  // Point 10
  {-800, 300, 9, 0, "C1", 190},  // Point 11
  {-800, 50, 0, 0, NULL, 465}, // Point 12
  {-1600, 50, 0, 0, NULL, 465},// Point 13
  {-1600, 300, 10, 0, "D1", 465}, // Point 14
  {-1600, 600, 11, 0, "D2", 465}, // Point 15
  {-1600, 900, 12, 0, "D3", 465}, // Point 16
  {-1600, 1450, 20, 0, NULL, 465} // Point 17 ， 停止点
};

static const uint8_t numPoints = sizeof(targetPoints) / sizeof(targetPoints[0]);
static uint8_t currentPointIndex = 0; // 当前点的索引
static const TickType_t waitTimeAtPoint = pdMS_TO_TICKS(100);



void Task1GoThrough()
{
  const float errorRange = 5; // 允许的误差范围 mm

  if (currentPointIndex >= numPoints) 
  {
    goThroughMode = TASK1_COMPLETE; // 所有点完成
  }

  
  float targetX = 0.0f;
  float targetY = 0.0f;
  float previousY = 0.0f;

  // 只有在索引有效时才获取目标点信息
  if (currentPointIndex < numPoints) 
  {
    targetX = targetPoints[currentPointIndex].x;
    targetY = targetPoints[currentPointIndex].y;
    previousY = currentPointIndex > 0 ? targetPoints[currentPointIndex - 1].y : 0;
    TIM9->CCR1 = targetPoints[currentPointIndex].ServoDegree; // 设置舵机角度
  }
  else if (numPoints > 0) 
  {
    // 如果已经完成，目标设为最后一个点以保持位置
    targetX = targetPoints[numPoints - 1].x;
    targetY = targetPoints[numPoints - 1].y;
  }

  switch (goThroughMode)
  {
    case TASK1_MOVING_X:
    {
      chassis.targetPosX = targetX;
      chassis.targetPosY = previousY;

      if (IsINRange(chassis.chassisDistance->distanceX, targetX, errorRange))
      {
        goThroughMode = TASK1_MOVING_Y;
      }
      break;
    }
    case TASK1_MOVING_Y:
    {
      chassis.targetPosX = targetX;
      chassis.targetPosY = targetY;

      if (IsINRange(chassis.chassisDistance->distanceY, targetY, errorRange))
      {
        goThroughMode = TASK1_REACH_PROCESSING;
      }
      break;
    }
    case TASK1_REACH_PROCESSING:
    {
      chassis.targetPosX = targetX; // Keep position
      chassis.targetPosY = targetY;

      vTaskDelay(waitTimeAtPoint); // 等待一段时间

      if (targetPoints[currentPointIndex].isQRPoint != 0)
      {
        xTaskNotify(Cmd_TaskHandle, NOTIFY_FROM_CAMERA_PROCESSING, eSetBits); // 通知摄像头处理二维码

        xTaskNotifyStateClear(NULL); // 清除通知状态

        goThroughMode = TASK1_WAITING_QR_RESULT; // 等待二维码处理完成
      }
      else
      {
        currentPointIndex++; // 移动到下一个点
        chassis.softStartFactor = 0.0f; // 重置软启动系数

        if (currentPointIndex >= numPoints)
          goThroughMode = TASK1_COMPLETE; // 所有点都已到达
        else
          goThroughMode = TASK1_MOVING_X; // 返回到移动到下一个点的状态
      }

      break;
    }
    case TASK1_WAITING_QR_RESULT:
    {
      // 等待二维码处理完成
      chassis.targetPosX = targetX;
      chassis.targetPosY = targetY;

      uint32_t ulNotifiedValue;
      // Wait specifically for the QR result ready notification from Cmd_Task
      BaseType_t xResult = xTaskNotifyWait(0x00,                    // Don't clear bits on entry
                                           NOTIFY_QR_RESULT_READY,  // Clear the result ready bit on exit
                                           &ulNotifiedValue,        // Store the notification value
                                           pdMS_TO_TICKS(2000));          // Wait for a specific duration

      if (xResult == pdPASS && (ulNotifiedValue & NOTIFY_QR_RESULT_READY) && (qrNumberInTask1 != targetPoints[currentPointIndex -1].qrNumber))
      {

        targetPoints[currentPointIndex].qrNumber = qrNumberInTask1; // 更新二维码内容
        SendQrNumber(&targetPoints[currentPointIndex]); // 发送二维码内容


        MicrophoneTypeDef sendMessage = 
        {
          .data = 
          {
              targetPoints[currentPointIndex].qrArea[0] - 'A' + 25,
              targetPoints[currentPointIndex].qrArea[1] - '0',
              targetPoints[currentPointIndex].qrNumber
          }
        };
        xQueueSend(microphoneQueue, &sendMessage, 0); // 向麦克风发送消息


        currentPointIndex++; // 移动到下一个点
        chassis.softStartFactor = 0.0f; // 重置软启动系数
        qrNumberInTask1 = 0; // 重置二维码处理标志

        if (currentPointIndex >= numPoints)
          goThroughMode = TASK1_COMPLETE; // 所有点都已到达
        else
          goThroughMode = TASK1_MOVING_X; // 返回到移动到下一个点的状态
      }
      else
      {
        xTaskNotify(Cmd_TaskHandle, NOTIFY_FROM_CAMERA_PROCESSING, eSetBits); // 通知摄像头处理二维码
        xTaskNotifyStateClear(NULL);
        vTaskDelay(pdMS_TO_TICKS(10)); // 等待二维码处理完成
      }
      break;
    }
    case TASK1_COMPLETE:
    {
      chassis.motorA->targetSpeed = 0;
      chassis.motorB->targetSpeed = 0;
      chassis.motorC->targetSpeed = 0;
      chassis.motorD->targetSpeed = 0;
      
      chassisMode = CHASSIS_TASK1_COMPLETE; // 返回到准备状态
      break;
    }
    default:
      break;
  }

}

void Task2GoThrough()
{

  switch (goThroughMode2)
  {
    case TASK2_NOTIFYING_CAMERA:
      xTaskNotify(Cmd_TaskHandle, NOTIFY_FROM_TASK2_TO_GET_QR_NUMBER, eSetBits); // 通知摄像头处理二维码
      goThroughMode2 = TASK2_WAITING_QR_RESULT; // 等待二维码结果准备好通知
      break;
    case TASK2_WAITING_QR_RESULT:
    {

      uint32_t ulNotifiedValue;
      BaseType_t xResult = xTaskNotifyWait(0x00, NOTIFY_QR_RESULT_READY, &ulNotifiedValue, pdMS_TO_TICKS(2000));
      if (xResult == pdPASS && (ulNotifiedValue & NOTIFY_QR_RESULT_READY) && qrNumberInTask2 != 0)
      {
        SendQrNumberToGetArea(qrNumberInTask2); // 发送二维码内容到2.4G
        xTaskNotifyStateClear(NULL);
      }
      goThroughMode2 = TASK2_NOTIFYING_2_4G; // 等待二维码结果准备好通知

      break;
    }
    case TASK2_NOTIFYING_2_4G:
    {
      if (qrNumberInTask2 != 0)
      {
        uint32_t ulNotifiedValue;
        BaseType_t xResult = xTaskNotifyWait(0x00, NOTIFY_QR_AREA_GOTTEN, &ulNotifiedValue, pdMS_TO_TICKS(2000));
        if (xResult == pdPASS && (ulNotifiedValue & NOTIFY_QR_AREA_GOTTEN) && qrNumberInTask2 != 0)
        {
          goThroughMode2 = TASK2_MOVING_X; // 等待二维码结果准备好通知
          xTaskNotifyStateClear(NULL);
        }
      }
      
      break;
    }
    case TASK2_MOVING_X:
    {
      chassis.targetPosX = qrAreaInTask2[0];
      chassis.targetPosY = 0;
      if (IsINRange(chassis.chassisDistance->distanceX, qrAreaInTask2[0], 5.0f))
      {
        goThroughMode2 = TASK2_MOVING_Y;
      }
      break;
    case TASK2_MOVING_Y:
      chassis.targetPosX = qrAreaInTask2[0];
      chassis.targetPosY = qrAreaInTask2[1];
      if (IsINRange(chassis.chassisDistance->distanceY, qrAreaInTask2[1], 5.0f))
      {
        goThroughMode2 = TASK2_COMPLETE;
      }
      break;
    }
    case TASK2_COMPLETE:
    {
      chassis.motorA->targetSpeed = 0;
      chassis.motorB->targetSpeed = 0;
      chassis.motorC->targetSpeed = 0;
      chassis.motorD->targetSpeed = 0;
      chassisMode = CHASSIS_TASK2_COMPLETE; // 返回到准备状态
      break;
    }
    default:
      break;
  }
}

void SendQrNumber(PointTypeDef *point)
{
  static uint8_t qrNumber_send[7]; // 静态分配，减少堆栈消耗
  qrNumber_send[0] = 0xAA;
  qrNumber_send[1] = 0x03;
  qrNumber_send[2] = point->qrArea[0];
  qrNumber_send[3] = point->qrArea[1];
  qrNumber_send[4] = point->qrNumber / 10 + '0';
  qrNumber_send[5] = point->qrNumber % 10 + '0';
  qrNumber_send[6] = 0xFF;

  HAL_UART_Transmit_DMA(CMD_2_4G_UART, qrNumber_send, sizeof(qrNumber_send));

  vTaskDelay(pdMS_TO_TICKS(50)); // 等待发送完成
}

// void SendMessageToMicrophone(uint8_t message)
// {
//   uint8_t messSend[5]; // 发送给麦克风的消息
//   messSend[0] = 0xAA; 
//   messSend[1] = 0x55; 
//   messSend[2] = message; 
//   messSend[3] = 0x55; 
//   messSend[4] = 0xAA; 

//   HAL_UART_Transmit_DMA(CMD_MICROPHONE_UART, messSend, sizeof(messSend));
// }

void SendReceiveSuccess()
{
  const uint8_t receiveSuccess[7] = {0xAA, 0x02, 0x11, 0x11, 0x11, 0x11, 0xFF}; // 接收成功指令
  HAL_UART_Transmit_DMA(CMD_2_4G_UART, receiveSuccess, PACKET_SIZE);
}

void SendQrNumberToGetArea(uint8_t qrNumber)
{
  uint8_t qrNumber_send[7]; // 静态分配，减少堆栈消耗
  qrNumber_send[0] = 0xAA;
  qrNumber_send[1] = 0x03;
  qrNumber_send[2] = qrNumber / 10 + '0';
  qrNumber_send[3] = qrNumber % 10 + '0';
  qrNumber_send[4] = 0x00; // 占位符
  qrNumber_send[5] = 0x00; // 占位符
  qrNumber_send[6] = 0xFF;

  HAL_UART_Transmit_DMA(CMD_2_4G_UART, qrNumber_send, sizeof(qrNumber_send));
}

uint8_t IsINRange(float nowValue, float targetValue, float range)
{
  if (nowValue > targetValue - range && nowValue < targetValue + range)
  {
    return 1;
  }
  return 0;
}

// void GetYaw(ChassisTypeDef* chassis)
// {
//   float yaw = 0.0f;
//   if (xQueueReceive(gyroYawQueue, &yaw, 0) == pdPASS)
//   {
//     // 只有在成功接收到新值时才更新 chassis->yaw
//     // VOFA_SendFireWater("yaw: %f\n", yaw); // 发送陀螺仪 yaw 数据
//     chassis->yaw = yaw;
//   }
// }