#include "Chassis_Task.h"


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


#define IS_DEBUG 0// 是否开启调试模式
ChassisModeTypeDef chassisMode = CHASSIS_TASK1_READY; // 0x01
static GoThroughTypeDef goThroughMode = TASK1_MOVING_X; // 任务1的状态
TaskHandle_t Chassis_TaskHandle = NULL;
void Chassis_Task(void *argument)
{
  const float softStartIncrement = 0.01f; // 软启动系数
  const float softStartMax = 1.0f; // 最大软启动系数

  

  Chassis_Init(&chassis);
  PID_Init(chassis.positionPidX, 0.6, 0.0, 0.0, 0.01, 200, 100, 2.5f, PID_MODE_POSITION);
  PID_Init(chassis.positionPidY, 0.6, 0.0, 0.0, 0.01, 200, 100, 2.5f, PID_MODE_POSITION);

  while (1)
  {
    #if IS_DEBUG
    
    chassis.chassisSpeed->vX = PIDCompute(chassis.positionPidX, chassis.chassisDistance->distanceX, chassis.targetPosX);
    chassis.chassisSpeed->vY = PIDCompute(chassis.positionPidY, chassis.chassisDistance->distanceY, chassis.targetPosY);


    // if (chassis.softStartFactor < softStartMax)
    // {
    //   chassis.softStartFactor += softStartIncrement;
    //   if (chassis.softStartFactor > softStartMax)
    //   {
    //     chassis.softStartFactor = softStartMax;
    //   }
    // } // 软启动系数逐渐增加
    // if (chassis.chassisSpeed->vX > 0.0f || chassis.chassisSpeed->vY > 0.0f)
    // {
    //   chassis.chassisSpeed->vX *= chassis.softStartFactor;
    //   chassis.chassisSpeed->vY *= chassis.softStartFactor;
    // }
    // else
    // {
    //   chassis.softStartFactor = 0.0f; // 重置软启动系数
    // }

    OmniWheelKinematics_FourWheel(&chassis);
    SetChassisSpeed_WithoutPID(&chassis);

    
    //   float sendData[] = {
    //     chassis.chassisSpeed->vX,
    //     chassis.chassisSpeed->vY,
    //     chassis.chassisSpeed->vW,
    //     chassis.motorA->speed,
    //     chassis.motorB->speed,
    //     chassis.motorC->speed,
    //     chassis.motorD->speed,
    //     chassis.chassisDistance->distanceX,
    //     chassis.chassisDistance->distanceY

    // };
    // VOFA_SendDataToVOFA(sendData, sizeof(sendData) / sizeof(sendData[0]));
    #else
    if (chassis.softStartFactor < softStartMax)
    {
      chassis.softStartFactor += softStartIncrement;
      if (chassis.softStartFactor > softStartMax)
      {
        chassis.softStartFactor = softStartMax;
      }
    } // 软启动系数逐渐增加
    GetDistance_FourWheel(&chassis);

    chassis.chassisSpeed->vX = PIDCompute(chassis.positionPidX, chassis.chassisDistance->distanceX, chassis.targetPosX);
    chassis.chassisSpeed->vY = PIDCompute(chassis.positionPidY, chassis.chassisDistance->distanceY, chassis.targetPosY);

    OmniWheelKinematics_FourWheel(&chassis);
    SetChassisSpeed_WithoutPID(&chassis);

    switch (chassisMode)
    {
      case CHASSIS_READY:
        
        break;
      case CHASSIS_TASK1_READY:
        // 处理任务1的逻辑
        SendReceiveSuccess();
        vTaskDelay(pdMS_TO_TICKS(100));

        
        goThroughMode = TASK1_MOVING_X;
        chassisMode = CHASSIS_TASK1_GO_THROUGH;
        break;
      case CHASSIS_TASK2_READY:
        // 处理任务2的逻辑


        break;
      case CHASSIS_TASK1_GO_THROUGH:
        {
          
          Task1GoThrough();
        }
        break;
      case CHASSIS_TASK1_COMPLETE:
        // 处理任务1完成的逻辑
        chassisMode = CHASSIS_READY; // 返回到准备状态
        break;
      default:
      
        break;
    }
    #endif
    


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static const PointTypeDef targetPoints[] = 
{
  // {-50, 0, 0, 0, NULL},    // Point 1
  {0, 300, 1, 0, "A1"},   // Point 2
  {0, 600, 1, 0, "A2"},   // Point 3
  {0, 900, 1, 0, "A3"},   // Point 4
  {0, 1350, 0, 0, NULL},  // Point 5
  {-760, 1350, 0, 0, NULL}, // Point 6
  {-760, 900, 1, 0, "B3"},  // Point 7
  {-760, 900, 1, 0, "C3"},  // Point 8
  {-760, 600, 1, 0, "C2"},  // Point 9
  {-760, 600, 1, 0, "B2"},  // Point 10
  {-760, 300, 1, 0, "B1"},  // Point 11
  {-760, 300, 1, 0, "C1"},  // Point 12
  {-760, -150, 0, 0, NULL}, // Point 13
  {-1600, -150, 0, 0, NULL},// Point 14
  {-1600, 300, 1, 0, "D1"}, // Point 15
  {-1600, 600, 1, 0, "D2"}, // Point 16
  {-1600, 900, 1, 0, "D3"}, // Point 17
  {-1600, 1350, 2, 0, NULL} // Point 18
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

  // 只有在索引有效时才获取目标点信息
  float targetX = 0.0f;
  float targetY = 0.0f;
  float previousY = 0.0f;
  uint8_t isQR = 0; // 当前点是否是二维码点

  if (currentPointIndex < numPoints) 
  {
    targetX = targetPoints[currentPointIndex].x;
    targetY = targetPoints[currentPointIndex].y;
    previousY = currentPointIndex > 0 ? targetPoints[currentPointIndex - 1].y : 0;
    isQR = targetPoints[currentPointIndex].isQRPoint;
  } else if (numPoints > 0) 
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
      // 到达当前点后处理
      
      vTaskDelay(waitTimeAtPoint); // 等待一段时间
      if (isQR == 1)
      {
        CameraProcessing(targetPoints[currentPointIndex]); // 处理二维码
        goThroughMode = TASK1_WAITING_QR_RESULT; // 等待二维码处理完成
      }
      else
      {
        currentPointIndex++; // 移动到下一个点
        chassis.softStartFactor = 0.0f; // 重置软启动系数
        if (currentPointIndex >= numPoints)
        {
          goThroughMode = TASK1_COMPLETE; // 所有点都已到达
        }
        else
        {
          goThroughMode = TASK1_MOVING_X; // 返回到移动到下一个点的状态
        }
      }

      break;
    }
    case TASK1_WAITING_QR_RESULT:
    {
      // 等待二维码处理完成
      chassis.targetPosX = targetX;
      chassis.targetPosY = targetY;

      if (qrNumberInCameraProcessing != 0) // 不为零则收到值
      {
        currentPointIndex++; // 移动到下一个点
        chassis.softStartFactor = 0.0f; // 重置软启动系数
        qrNumberInCameraProcessing = 0; // 重置二维码处理标志
        if (currentPointIndex >= numPoints)
        {
          goThroughMode = TASK1_COMPLETE; // 所有点都已到达
        }
        else
        {
          goThroughMode = TASK1_MOVING_X; // 返回到移动到下一个点的状态
        }
      }
      else
      {
        vTaskDelay(pdMS_TO_TICKS(10)); // 等待二维码处理完成
      }
      break;
    }
    case TASK1_COMPLETE:
    {
      // 所有点都已到达，处理完成逻辑
      // 例如，停止电机或发送完成信号
      chassisMode = CHASSIS_TASK1_COMPLETE; // 返回到准备状态
      break;
    }
    default:
      break;
  }

}

uint8_t qrNumberInCameraProcessing = 0;;
void CameraProcessing(PointTypeDef point)
{
  BaseType_t result = xTaskNotify(Cmd_TaskHandle,
                                  NOTIFY_FROM_CAMERA_PROCESSING,
                                  eSetBits); // 通知摄像头处理任务
  if (result == pdPASS)
  {
    if (qrNumberInCameraProcessing != 0)
    {
      uint8_t qrNumber[2] = {0, 0};
      qrNumber[0] = qrNumberInCameraProcessing / 10;
      qrNumber[1] = qrNumberInCameraProcessing % 10;
      uint8_t qrArea[2] = {0, 0};
      qrArea[0] = point.qrArea[0];
      qrArea[1] = point.qrArea[1];

      SendQrNumber(qrNumber, qrArea); // 发送二维码指令
    }
  }
}

void SendQrNumber(uint8_t *Number, uint8_t *Area)
{
  const uint8_t qrNumber[7] = {0xAA, 0x03, Area[0], Area[1], Number[0], Number[1], 0xFF}; // 二维码指令
  HAL_UART_Transmit_DMA(CMD_2_4G_UART, qrNumber, PACKET_SIZE);
}

void SendReceiveSuccess()
{
  const uint8_t receiveSuccess[7] = {0xAA, 0x02, 0x11, 0x11, 0x11, 0x11, 0xFF}; // 接收成功指令
  HAL_UART_Transmit_DMA(CMD_2_4G_UART, receiveSuccess, PACKET_SIZE);
}


uint8_t IsINRange(float nowValue, float targetValue, float range)
{
  if (nowValue > targetValue - range && nowValue < targetValue + range)
  {
    return 1;
  }
  return 0;
}


/**
 * @brief 设置目标位置并检查是否两个轴都已到达指定范围
 * @param targetX 目标X坐标
 * @param targetY 目标Y坐标
 * @param range 允许的误差范围
 * @return 1: 已到达目标; 0: 未到达目标
 */
uint8_t MoveToTargetAndCheck(float targetX, float targetY, float range)
{
  // 设置目标位置供PID控制器使用
  chassis.targetPosX = targetX;
  chassis.targetPosY = targetY;

  // 检查当前位置是否在目标范围内
  uint8_t xReached = IsINRange(chassis.chassisDistance->distanceX, targetX, range);
  uint8_t yReached = IsINRange(chassis.chassisDistance->distanceY, targetY, range);

  // 如果两个轴都到达，返回1
  if (xReached && yReached)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}