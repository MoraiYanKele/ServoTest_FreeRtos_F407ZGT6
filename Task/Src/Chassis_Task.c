#include "Chassis_Task.h"

TaskHandle_t Chassis_TaskHandle = NULL;
extern ChassisTypeDef chassis;

void TIM5CallBack_Task() // 100Hz, 10ms
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  Chassis_GetEncoder(&chassis);
  SetChassisSpeed(&chassis);
  GetDistance(&chassis);
  
  vTaskNotifyGiveFromISR(Motor1_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor2_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor3_TaskHandle, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(Motor4_TaskHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
} 

ChassisModeTypeDef chassisMode = CHASSIS_TASK1_READY; // 0x01
static GoThroughTypeDef goThroughMode = TASK1_MOVING_X; // 任务1的状态

void Chassis_Task(void *argument)
{
  const float softStartIncrement = 0.01f; // 软启动系数
  const float softStartMax = 1.0f; // 最大软启动系数

  Chassis_Init(&chassis);
  PID_Init(chassis.positionPidX, 1.5, 0.0, 0.0, 0.01, 200, 100, 0.0f, PID_MODE_POSITION);
  PID_Init(chassis.positionPidY, 1.5, 0.0, 0.0, 0.01, 200, 100, 0.0f, PID_MODE_POSITION);
  VOFA_RegisterData_float("chassisX", &chassis.targetPosX);
  VOFA_RegisterData_float("chassisY", &chassis.targetPosY);
  while (1)
  {
    if (chassis.softStartFactor < softStartMax)
    {
      chassis.softStartFactor += softStartIncrement;
      if (chassis.softStartFactor > softStartMax)
      {
        chassis.softStartFactor = softStartMax;
      }
    } // 软启动系数逐渐增加

    switch (chassisMode)
    {
      case CHASSIS_READY:
        
        break;
      case CHASSIS_TASK1_READY:
        // 处理任务1的逻辑
        vTaskDelay(pdMS_TO_TICKS(100));

        SendReceiveSuccess();
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
    // float sendData[] = {
    //   chassis.motorA->position,
    //   chassis.motorB->position,
    //   chassis.motorC->position,
    //   chassis.motorD->position,
    //   chassis.chassisDistance->distanceX,
    //   chassis.chassisDistance->distanceY,
    //   chassis.targetPosX,
    //   chassis.targetPosY,
    // };
    // VOFA_SendDataToVOFA(sendData, sizeof(sendData) / sizeof(sendData[0]));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static const PointTypeDef targetPoints[] = 
{
  {50, 0, 0, 0, NULL},    // Point 1
  {50, 300, 1, 0, "A1"},   // Point 2
  {50, 600, 1, 0, "A2"},   // Point 3
  {50, 900, 1, 0, "A3"},   // Point 4
  {50, 1350, 0, 0, NULL},  // Point 5
  {760, 1350, 0, 0, NULL}, // Point 6
  {760, 900, 1, 0, "B3"},  // Point 7
  {890, 900, 1, 0, "C3"},  // Point 8
  {890, 600, 1, 0, "C2"},  // Point 9
  {760, 600, 1, 0, "B2"},  // Point 10
  {760, 300, 1, 0, "B1"},  // Point 11
  {890, 300, 1, 0, "C1"},  // Point 12
  {890, -150, 0, 0, NULL}, // Point 13
  {1600, -150, 0, 0, NULL},// Point 14
  {1600, 300, 1, 0, "D1"}, // Point 15
  {1600, 600, 1, 0, "D2"}, // Point 16
  {1600, 900, 1, 0, "D3"}, // Point 17
  {1600, 1350, 2, 0, NULL} // Point 18
};

static const uint8_t numPoints = sizeof(targetPoints) / sizeof(targetPoints[0]);
static uint8_t currentPointIndex = 0; // 当前点的索引
static const TickType_t waitTimeAtPoint = pdMS_TO_TICKS(100);



void Task1GoThrough()
{
  const float errorRange = 0.5; // 允许的误差范围

  if (currentPointIndex >= numPoints) 
  {
    goThroughMode = TASK1_COMPLETE; // 所有点完成
  }


    float targetX = targetPoints[currentPointIndex].x;
    float targetY = targetPoints[currentPointIndex].y;
  
    float previousY = currentPointIndex > 0 ? targetPoints[currentPointIndex - 1].y : 0;

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
      if (targetPoints[currentPointIndex].isQRPoint == 1)
      {
        // 处理二维码逻辑
 
        // while () // 等待二维码处理完成
        // {

        // }
      }
      currentPointIndex++; // 移动到下一个点
      if (currentPointIndex >= numPoints)
      {
        goThroughMode = TASK1_COMPLETE; // 所有点都已到达
      }
      else
      {
        goThroughMode = TASK1_MOVING_X; // 返回到移动到下一个点的状态
      }

      chassis.softStartFactor = 0.0f; // 重置软启动系数
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

const uint8_t receiveSuccess[7] = {0xAA, 0x02, 0x11, 0x11, 0x11, 0x11, 0xFF}; // 接收成功指令
void SendReceiveSuccess()
{
  HAL_UART_Transmit_DMA(&huart3, receiveSuccess, PACKET_SIZE);
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