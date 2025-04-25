#include "Chassis.h"

ChassisSpeedTypeDef chassisSpeed = 
{
  .vX = 0,
  .vY = 0,
  .vW = 0
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


PIDControllerTypedef positionPidForX;
PIDControllerTypedef positionPidForY;

ChassisTypeDef chassis = 
{
  .motorA = &motor1,
  .motorB = &motor2,
  .motorC = &motor3,
  .motorD = &motor4,
  
  .chassisSpeed = &chassisSpeed,
  .wheelSpeed = &wheelSpeed,
  .chassisDistance = &chassisDistance,

  .positionPidX = &positionPidForX,
  .positionPidY = &positionPidForY
};

void Chassis_Init(ChassisTypeDef* chassis)
{

  chassis->targetPosX = 0;
  chassis->targetPosY = 0;

  chassis->motorA->targetSpeed = 0;
  chassis->motorB->targetSpeed = 0;
  chassis->motorC->targetSpeed = 0;
  chassis->motorD->targetSpeed = 0;

  chassis->motorA->position = 0;
  chassis->motorB->position = 0;
  chassis->motorC->position = 0;
  chassis->motorD->position = 0;

  chassis->chassisSpeed->vX = 0;
  chassis->chassisSpeed->vY = 0;
  chassis->chassisSpeed->vW = 0;

  chassis->wheelSpeed->vA = 0;
  chassis->wheelSpeed->vB = 0;
  chassis->wheelSpeed->vC = 0;
  chassis->wheelSpeed->vD = 0;
}

void OmniWheelKinematics(ChassisTypeDef* chassis)
{
  float vX = chassis->chassisSpeed->vX;
  float vY = chassis->chassisSpeed->vY;
  float vW = chassis->chassisSpeed->vW;

  chassis->wheelSpeed->vA = -vX;
  chassis->wheelSpeed->vA = -vY;
  chassis->wheelSpeed->vA = vX;
  chassis->wheelSpeed->vA = vY;
}

void SetChassisSpeed(ChassisTypeDef* chassis)
{
  chassis->motorA->targetSpeed = -PIDCompute(chassis->positionPidX, -chassis->motorA->position, chassis->targetPosX);
  chassis->motorB->targetSpeed = -PIDCompute(chassis->positionPidY, -chassis->motorB->position, chassis->targetPosY);
  chassis->motorC->targetSpeed = PIDCompute(chassis->positionPidX, chassis->motorC->position, chassis->targetPosX);
  chassis->motorD->targetSpeed = PIDCompute(chassis->positionPidY, chassis->motorD->position, chassis->targetPosY);
}

void GetDistance(ChassisTypeDef* chassis)
{
  chassis->chassisDistance->distanceX = (-chassis->motorA->position + chassis->motorC->position) / 2;
  chassis->chassisDistance->distanceY = (-chassis->motorB->position + chassis->motorD->position) / 2; 
}
void Chassis_GetEncoder(ChassisTypeDef* chassis)
{
  GetEncoder(chassis->motorA);
  GetEncoder(chassis->motorB);
  GetEncoder(chassis->motorC);
  GetEncoder(chassis->motorD);
}