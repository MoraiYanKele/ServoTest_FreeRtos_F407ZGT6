#include "Chassis.h"

ChassisSpeedTypeDef chassisSpeed = 
{
  .vX = 0,
  .vY = 0,
};

WheelSpeedTypedef wheelSpeed = 
{
  .vA = 0,
  .vB = 0,
  .vC = 0,
  .vD = 0
};

ChassisDistanceTypeDef chassisDistance = 
{
  .distanceX = 0,
  .distanceY = 0
};

void Chassis_Init()
{
  chassisSpeed.vX = 0;
  chassisSpeed.vY = 0;

  chassisDistance.distanceX = 0;
  chassisDistance.distanceY = 0;
  
  wheelSpeed.vA = 0;
  wheelSpeed.vB = 0;
  wheelSpeed.vC = 0;
  wheelSpeed.vD = 0;
}

void OmniWheelKinematics(ChassisSpeedTypeDef* input, WheelSpeedTypedef* output)
{
  float vX = input->vX;
  float vY = input->vY;

  output->vA = -vX;
  output->vB = -vY;
  output->vC = vX;
  output->vD = vY;
}

void SetChassisSpeed(WheelSpeedTypedef* wheelSpeed)
{
  targetSpeed1 = wheelSpeed->vA;
  targetSpeed2 = wheelSpeed->vB;
  targetSpeed3 = wheelSpeed->vC;
  targetSpeed4 = wheelSpeed->vD;
}

void GetDistance(ChassisDistanceTypeDef* chassisDistance)
{
  chassisDistance->distanceX = (-position1 + position3) / 2; // 左前 + 右后
  chassisDistance->distanceY = (-position2 + position4) / 2; // 右前 + 左后
}
