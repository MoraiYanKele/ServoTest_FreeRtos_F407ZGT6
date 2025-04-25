#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "Motor.h"
#include "pid.h"
#include "VOFA.h"

extern MotorTypeDef motor1;
extern MotorTypeDef motor2;
extern MotorTypeDef motor3;
extern MotorTypeDef motor4;

extern TaskHandle_t MotorPIDTest_TaskHandle;
extern TaskHandle_t MotorEncoder_TaskHandle;

extern TaskHandle_t Motor1_TaskHandle;
extern TaskHandle_t Motor2_TaskHandle;
extern TaskHandle_t Motor3_TaskHandle;
extern TaskHandle_t Motor4_TaskHandle;



void MotorPIDTest_Task(void *argument);
// void MotorEncoder_Task(void *argument);
void Moto1_Task(void *argument);
void Moto2_Task(void *argument);
void Moto3_Task(void *argument);
void Moto4_Task(void *argument);

void SpeedPID_Init(MotorTypeDef *motor, float kp, float ki, float kd);

#endif //  __MOTORPIDTEST_TASK_H__

