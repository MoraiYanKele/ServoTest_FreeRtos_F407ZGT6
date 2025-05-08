#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"
#include "Motor.h"
#include "Motor_Task.h"
#include "ProjectHeader.h"
#include "chassis_Task.h"

#define LIMIT_MAGNITUDE(value, low, high) \
        ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define  MOTROR_SPEED_LIMIT     100.0f // 电机最大速度

#define COS_45      0.70710678118
#define L           0.11f

typedef struct 
{
    float vX;   // x方向速度（左右）
    float vY;   // y方向速度（前后）
    float vW;   // 角速度（旋转）
} ChassisSpeedTypeDef;

typedef struct 
{
  float distanceX;
  float distanceY;
}ChassisDistanceTypeDef;


typedef struct 
{
    float vA;   // 轮A（左前）
    float vB;   // 轮B（右前）
    float vC;   // 轮C（左后）
    float vD;   // 轮D（右后）
} WheelSpeedTypedef;



typedef struct ChassisTypeDef
{
  MotorTypeDef *motorA;
  MotorTypeDef *motorB;
  MotorTypeDef *motorC;
  MotorTypeDef *motorD;

  ChassisSpeedTypeDef *chassisSpeed;
  WheelSpeedTypedef *wheelSpeed;
  ChassisDistanceTypeDef *chassisDistance;

  PIDControllerTypedef *positionPidX;
  PIDControllerTypedef *positionPidY;
  PIDControllerTypedef *positionPidW;

  float targetPosX;
  float targetPosY;
  float targetPosW; // 目标角度

  float softStartFactor; // 软启动系数

  float yaw; // 陀螺仪 yaw 角度
    
} ChassisTypeDef;

void Chassis_Init(ChassisTypeDef* chassis);
void OmniWheelKinematics(ChassisTypeDef* chassis);
void OmniWheelKinematics_FourWheel(ChassisTypeDef* chassis);
void SetChassisSpeed(ChassisTypeDef* chassis);
void SetChassisSpeed_WithoutPID(ChassisTypeDef* chassis);
void GetDistance(ChassisTypeDef* chassis);
void GetDistance_FourWheel(ChassisTypeDef* chassis);
void Chassis_GetEncoder(ChassisTypeDef* chassis);


#endif