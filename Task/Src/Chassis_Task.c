#include "Chassis_Task.h"

TaskHandle_t Chassis_TaskHandle = NULL;

PIDControllerTypedef positionPidForX;
PIDControllerTypedef positionPidForY;
PIDControllerTypedef positionPidForW;

float targetPosX = 0;
float targetPosY = 0;
float targetPosW = 0;

void Chassis_Task(void *argument)
{
  Chassis_Init();
  PID_Init(&positionPidForX, 1, 0, 0);
  PID_SetOutputLimits(&positionPidForX, -150, 150);
  PID_SetDeadBand(&positionPidForX, 0.5);
  PID_SetIntegralSeparationThreshold(&positionPidForX, 400);
  PID_SetIntegralLimits(&positionPidForX, -50, 50);
  PID_SetSampleTime(&positionPidForX, 0.01);
  PID_SetMode(&positionPidForX, PID_MODE_POSITION);


  PID_Init(&positionPidForY, 1, 0, 0);
  PID_SetOutputLimits(&positionPidForY, -150, 150);
  PID_SetDeadBand(&positionPidForY, 0.5);
  PID_SetIntegralSeparationThreshold(&positionPidForY, 400);
  PID_SetIntegralLimits(&positionPidForY, -50, 50);
  PID_SetSampleTime(&positionPidForY, 0.01);
  PID_SetMode(&positionPidForY, PID_MODE_POSITION);

  if (yaw != 0)
  {
    targetPosW = (float)yaw;
  }
  else
  {
    targetPosW = 0;
  }

  while (1)
  {
    targetSpeed1 = -PIDCompute(&positionPidForX, -position1, targetPosX);
    targetSpeed2 = -PIDCompute(&positionPidForY, -position2, targetPosY);
    targetSpeed3 = PIDCompute(&positionPidForX, position3, targetPosX);
    targetSpeed4 = PIDCompute(&positionPidForY, position4, targetPosY);
    chassisSpeed.vY = PIDCompute(&positionPidForY, chassisDistance.distanceY, targetPosY);
    // chassisSpeed.vW = PIDCompute(&positionPidForW, (float)yaw, targetPosW);
    // OmniWheelKinematics(&chassisSpeed, &wheelSpeed);
    // SetChassisSpeed(&wheelSpeed);
    GetDistance(&chassisDistance);


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}