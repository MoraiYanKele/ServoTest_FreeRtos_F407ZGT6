#include "Motor_Task.h"

TaskHandle_t MotorPIDTest_TaskHandle = NULL;
TaskHandle_t Motor1_TaskHandle = NULL;
TaskHandle_t Motor2_TaskHandle = NULL;
TaskHandle_t Motor3_TaskHandle = NULL;
TaskHandle_t Motor4_TaskHandle = NULL;


TaskHandle_t MotorEncoder_TaskHandle = NULL;

PIDControllerTypedef speedPidForMotor1;
PIDControllerTypedef speedPidForMotor2;
PIDControllerTypedef speedPidForMotor3;
PIDControllerTypedef speedPidForMotor4;



MotorTypeDef motor1 = 
{
  .IN_PORTA = BIN1_2_GPIO_Port,
  .GPIO_PinA = BIN1_2_Pin,
  .IN_PORTB = BIN2_2_GPIO_Port,
  .GPIO_PinB = BIN2_2_Pin,

  .htimPWM = &htim1,
  .PWMTIM_CHANNEL = TIM_CHANNEL_4,

  .htimEncoder = &htim4,

  .speedPid = &speedPidForMotor1
};

MotorTypeDef motor2 = 
{
  .IN_PORTA = AIN2_2_GPIO_Port,
  .GPIO_PinA = AIN2_2_Pin,
  .IN_PORTB = AIN1_2_GPIO_Port,
  .GPIO_PinB = AIN1_2_Pin,

  .htimPWM = &htim1,
  .PWMTIM_CHANNEL = TIM_CHANNEL_3,

  .htimEncoder = &htim8,

  .speedPid = &speedPidForMotor2
};

MotorTypeDef motor3 = 
{
  .IN_PORTA = AIN2_1_GPIO_Port,
  .GPIO_PinA = AIN2_1_Pin,
  .IN_PORTB = AIN1_1_GPIO_Port,
  .GPIO_PinB = AIN1_1_Pin,

  .htimPWM = &htim1,
  .PWMTIM_CHANNEL = TIM_CHANNEL_1,

  .htimEncoder = &htim3,

  .speedPid = &speedPidForMotor3
};

MotorTypeDef motor4 = 
{
  .IN_PORTA = BIN1_1_GPIO_Port,
  .GPIO_PinA = BIN1_1_Pin,
  .IN_PORTB = BIN2_1_GPIO_Port,
  .GPIO_PinB = BIN2_1_Pin,

  .htimPWM = &htim1,
  .PWMTIM_CHANNEL = TIM_CHANNEL_2,

  .htimEncoder = &htim2,

  .speedPid = &speedPidForMotor4
};

// float targetSpeed1 = 0.0f;
// float targetSpeed2 = 0.0f;
// float targetSpeed3 = 0.0f;
// float targetSpeed4 = 0.0f;

// float position1 = 0.0f;
// float position2 = 0.0f;
// float position3 = 0.0f;
// float position4 = 0.0f;




void MotorPIDTest_Task(void *argument)
{
  /* Infinite loop */


  vTaskDelay(pdMS_TO_TICKS(10));
  
  while (1)
  {

  }
}


void Moto1_Task(void *argument)
{
  Motor_Init(&motor1);
  SpeedPID_Init(&motor1, 1.5, 10, 0.0);
  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      motor1.speed = (float)(motor1.encoder) * SPEED_FACTOR;
      motor1.position += (float)(motor1.encoder) * DISTENCE_FACTOR; // 计算位置，单位为mm
      PIDSetSpeed(&motor1, motor1.targetSpeed);
    };
  }
}

void Moto2_Task(void *argument)
{
  // 初始化绝对时间变量
  Motor_Init(&motor2);
  SpeedPID_Init(&motor2, 1.5, 10, 0.0);
  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      motor2.speed = (float)(motor2.encoder) * SPEED_FACTOR;
      motor1.position += (float)(motor2.encoder) * DISTENCE_FACTOR; // 计算位置，单位为mm
      PIDSetSpeed(&motor2, motor2.targetSpeed);
    };
  }
}

void Moto3_Task(void *argument)
{
  // 初始化绝对时间变量
  Motor_Init(&motor3);
  SpeedPID_Init(&motor3, 1.5, 10, 0.0);
  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      motor3.speed = (float)(motor3.encoder) * SPEED_FACTOR;
      motor1.position += (float)(motor3.encoder) * DISTENCE_FACTOR; // 计算位置，单位为mm
      PIDSetSpeed(&motor3, motor3.targetSpeed);
    };
  }
}

void Moto4_Task(void *argument)
{
  // 初始化绝对时间变量
  Motor_Init(&motor4);
  SpeedPID_Init(&motor4, 1.5, 10, 0.0);
  while (1)
  {
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notificationValue > 0)
    {
      motor4.speed = (float)(motor4.encoder) * SPEED_FACTOR;
      motor1.position += (float)(motor4.encoder) * DISTENCE_FACTOR; // 计算位置，单位为mm
      PIDSetSpeed(&motor4, motor4.targetSpeed);
    };
  }
}


void SpeedPID_Init(MotorTypeDef *motor, float kp, float ki, float kd)
{
  PID_Init(motor->speedPid, kp, ki, kd, 0.01, 100, 25, 0.0f, PID_MODE_POSITION);
}
