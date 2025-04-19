#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pid.h"
#include "freertos.h"
#include "task.h"


#define LIMIT_MAGNITUDE(value, low, high) \
        ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

#define MOTOR_SPEED_LIMIT       00.0f  // 电机最大速度
#define SPEED_FACTOR            4.6875f // 
#define DISTENCE_FACTOR         0.068067 // m/min

typedef struct MotorTypeDef
{
    GPIO_TypeDef *IN_PORTA;
    GPIO_TypeDef *IN_PORTB;
    uint16_t GPIO_PinA;
    uint16_t GPIO_PinB;

    TIM_HandleTypeDef *htimPWM;
    uint32_t PWMTIM_CHANNEL;

    TIM_HandleTypeDef *htimEncoder;

    // 为了设定pwm的速度
    float speed;  // 单位rpm,  电机最大速度为285rpm
    float PWMForSpeed; // 设定的速度
    int lastSpeed;

    // 编码器在定时器周期内的计数，可以抽象理解为速度？？？
    short encoder;

    PIDControllerTypedef *speedPid;
    
    // float (*GetRealisticSpeed)(struct MotorTypeDef *motor); // 新增的返回float的函数指针
    // void (*PIDSetSpeed)(struct MotorTypeDef *motor, float targetSpeed);
    // void (*SetSpeed)(struct MotorTypeDef *motor, float speed);
    // void (*GetEncoder)(struct MotorTypeDef *motor);

} MotorTypeDef;




float GetRealisticSpeed(MotorTypeDef *self); 
void Motor_Init(MotorTypeDef *self);
void GetEncoder(MotorTypeDef *self);
void SetSpeed(MotorTypeDef *self, float speedPWM);
void PIDSetSpeed(MotorTypeDef *self, float targetSpeed);


#endif