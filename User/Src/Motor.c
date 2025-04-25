#include "Motor.h"




void Motor_Init(MotorTypeDef *self)
{
    self->speed = 0;
    self->lastSpeed = 0;
    self->encoder = 0;
    self->PWMForSpeed = 0;
    self->targetSpeed = 0;
    self->position = 0;

    // self->SetSpeed = SetSpeed;
    // self->GetEncoder = GetEncoder;
    // self->PIDSetSpeed = PIDSetSpeed;
    // self->GetRealisticSpeed = GetRealisticSpeed;

    HAL_TIM_Encoder_Start(self->htimEncoder, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(self->htimEncoder, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(self->htimPWM, self->PWMTIM_CHANNEL);

    HAL_GPIO_WritePin(self->IN_PORTA, self->GPIO_PinA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(self->IN_PORTB, self->GPIO_PinB, GPIO_PIN_RESET);
}

void GetEncoder(MotorTypeDef *self)
{
  
  self->encoder = (short)(__HAL_TIM_GET_COUNTER(self->htimEncoder));
  __HAL_TIM_SET_COUNTER(self->htimEncoder, 0);
}

void SetSpeed(MotorTypeDef *self, float speedPWM)
{
 
    int SpeedPWM_int = LIMIT_MAGNITUDE((int)speedPWM, -100, 100);
    if (SpeedPWM_int * self->lastSpeed >= 0)
    {
        HAL_GPIO_WritePin(self->IN_PORTA, self->GPIO_PinA, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(self->IN_PORTB, self->GPIO_PinB, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(5)); // 在freertos中更换成非阻塞延时
        // HAL_Delay(5); // 在freertos中更换成非阻塞延时
    }

    if (SpeedPWM_int >= 0) // 正转
    {
        HAL_GPIO_WritePin(self->IN_PORTA, self->GPIO_PinA, GPIO_PIN_SET);
        HAL_GPIO_WritePin(self->IN_PORTB, self->GPIO_PinB, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(self->htimPWM, self->PWMTIM_CHANNEL, SpeedPWM_int);
        // HAL_Delay(5);
    }
    else // 反转
    {
        HAL_GPIO_WritePin(self->IN_PORTA, self->GPIO_PinA, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(self->IN_PORTB, self->GPIO_PinB, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(self->htimPWM, self->PWMTIM_CHANNEL, -SpeedPWM_int);
        // HAL_Delay(5);
    }
    self->lastSpeed = SpeedPWM_int;
    
}

/**
 * @brief 获取电机的实际速度
 * 
 * @param self 电机对象指针
 * @return float 返回电机的实际速度，经过单位换算后的值
 * 
 * @note 该函数读取电机的原始速度数据并应用转换因子，将其转换为工程单位
 *       如果传入空指针，则返回0
 * 
 * @warning 当前实现有错误：计算了speedReal但返回了self->speed
 */
float GetRealisticSpeed(MotorTypeDef *self)
{
    if (self == NULL)
       return 0;
    float speedReal = self->speed * SPEED_FACTOR;
    return speedReal;
}

/**
 * @brief 使用PID控制器设置电机速度
 * @param self 电机对象指针
 * @param targetSpeed 目标速度
 * @note 此函数首先通过PID控制器计算适当的PWM值，然后调用SetSpeed函数设置电机速度
 */
void PIDSetSpeed(MotorTypeDef *self, float targetSpeed)
{
    if (self == NULL || self->speedPid == NULL)
        return;
       
    PID_SetOutputLimits(self->speedPid, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);

    self->PWMForSpeed = PIDCompute(self->speedPid, self->speed, targetSpeed);

    // 设置电机速度
    SetSpeed(self, self->PWMForSpeed);
}