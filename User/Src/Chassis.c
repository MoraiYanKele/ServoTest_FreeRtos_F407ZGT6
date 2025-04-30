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

  chassis->chassisDistance->distanceX = 0;
  chassis->chassisDistance->distanceY = 0;

  chassis->softStartFactor = 0.0f; // 软启动系数
}

void OmniWheelKinematics(ChassisTypeDef* chassis)
{
  float vX = chassis->chassisSpeed->vX;
  float vY = chassis->chassisSpeed->vY;
  float vW = chassis->chassisSpeed->vW;

  chassis->wheelSpeed->vA = -vX;
  chassis->wheelSpeed->vB = -vY;
  chassis->wheelSpeed->vC = vX;
  chassis->wheelSpeed->vD = vY;
}

void OmniWheelKinematics_FourWheel(ChassisTypeDef* chassis)
{
  float vx_term = chassis->chassisSpeed->vX * COS_45;
  float vy_term = chassis->chassisSpeed->vY * COS_45;
  float w_term  = L * chassis->chassisSpeed->vW;
  
  chassis->wheelSpeed->vA = -vx_term + vy_term + w_term; // 左前轮
  chassis->wheelSpeed->vB = -vx_term - vy_term + w_term;  // 右前轮
  chassis->wheelSpeed->vC = vx_term - vy_term + w_term; // 左后轮
  chassis->wheelSpeed->vD = vx_term + vy_term + w_term;  // 右后轮

}

void SetChassisSpeed(ChassisTypeDef* chassis)
{
  float pidoutputTargetSpeedA = PIDCompute(chassis->positionPidX, chassis->motorA->position, chassis->targetPosX);
  float pidoutputTargetSpeedB = -PIDCompute(chassis->positionPidY, -chassis->motorB->position, chassis->targetPosY);
  float pidoutputTargetSpeedC = -PIDCompute(chassis->positionPidX, -chassis->motorC->position, chassis->targetPosX);
  float pidoutputTargetSpeedD = PIDCompute(chassis->positionPidY, chassis->motorD->position, chassis->targetPosY);

  // chassis->motorA->targetSpeed = pidoutputTargetSpeedA * chassis->softStartFactor;
  // chassis->motorB->targetSpeed = pidoutputTargetSpeedB * chassis->softStartFactor;
  // chassis->motorC->targetSpeed = pidoutputTargetSpeedC * chassis->softStartFactor;
  // chassis->motorD->targetSpeed = pidoutputTargetSpeedD * chassis->softStartFactor;

  chassis->motorA->targetSpeed = pidoutputTargetSpeedA;
  chassis->motorB->targetSpeed = pidoutputTargetSpeedB;
  chassis->motorC->targetSpeed = pidoutputTargetSpeedC;
  chassis->motorD->targetSpeed = pidoutputTargetSpeedD;

}

void SetChassisSpeed_WithoutPID(ChassisTypeDef* chassis)
{
  chassis->motorA->targetSpeed = chassis->wheelSpeed->vA * chassis->softStartFactor;
  chassis->motorB->targetSpeed = chassis->wheelSpeed->vB * chassis->softStartFactor;
  chassis->motorC->targetSpeed = chassis->wheelSpeed->vC * chassis->softStartFactor;
  chassis->motorD->targetSpeed = chassis->wheelSpeed->vD * chassis->softStartFactor;
}

void GetDistance(ChassisTypeDef* chassis)
{
  chassis->chassisDistance->distanceX = (chassis->motorA->position - chassis->motorC->position) / 2;
  chassis->chassisDistance->distanceY = (-chassis->motorB->position + chassis->motorD->position) / 2; 
}

void GetDistance_FourWheel(ChassisTypeDef* chassis)
{
  float posA = chassis->motorA->position;
  float posB = chassis->motorB->position;
  float posC = chassis->motorC->position;
  float posD = chassis->motorD->position;

  // 根据正向运动学公式计算底盘位移
  chassis->chassisDistance->distanceX =(-posA - posB + posC + posD) / (4 * COS_45);
  chassis->chassisDistance->distanceY = ( posA - posB - posC + posD) / (4 * COS_45);
}
void Chassis_GetEncoder(ChassisTypeDef* chassis)
{
  GetEncoder(chassis->motorA);
  GetEncoder(chassis->motorB);
  GetEncoder(chassis->motorC);
  GetEncoder(chassis->motorD);
}