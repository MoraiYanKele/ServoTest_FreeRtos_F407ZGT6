#include "Chassis_Task.h"

TaskHandle_t Chassis_TaskHandle = NULL;

void Chassis_Task(void *argument)
{
  Chassis_Init();
  chassisSpeed.vX = 100;
  while (1)
  {
    OmniWheelKinematics(&chassisSpeed, &wheelSpeed);
    SetChassisSpeed(&wheelSpeed);
    GetDistance(&chassisDistance);
    if (chassisDistance.distanceX > 1000)
    {
      chassisSpeed.vX = 0;
    }
  }
}