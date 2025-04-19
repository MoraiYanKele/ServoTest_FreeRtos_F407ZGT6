#include "pid.h"
#include "VOFA.h"
#include <math.h>



/**
 * @brief  初始化pid结构体
 * @param  PIDControllerTypedef *pid: 要初始化的pid结构体的地址
 * @param  float kp: 初始的比例增益
 * @param  float ki: 初始的积分增益
 * @param  float kd: 初始的微分增益
 */
void PID_Init(PIDControllerTypedef *pid, float Kp, float Ki, float Kd)
{
  if (pid == NULL)
    return;
      
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  
  pid->targetValue = 0.0f;
  pid->error = 0.0f;
  pid->previous_error = 0.0f;
  pid->prev_prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->derivative = 0.0f;
  pid->filteredDerivative = 0.0f;
  
  // 默认输出限制
  pid->outputMax = 1000.0f;
  pid->outputMin = -1000.0f;
  
  // 默认积分限制
  pid->integralMax = 500.0f;
  pid->integralMin = -500.0f;
  
  // 默认无死区
  pid->deadBand = 0.0f;
  
  // 默认积分分离阈值
  pid->integralSeparationThreshold = 200.0f;
  
  // 默认微分滤波系数
  pid->derivativeFilterFactor = 0.8f;
  
  // 默认位置式PID
  pid->mode = PID_MODE_POSITION;

  // 默认采样时间设置为0.01秒(100Hz)
  pid->sampleTime = 0.01f;
  pid->invSampleTime = 100.0f;  // 1/0.01 = 100

  pid->previous_measurement = 0.0f;
}

/**
 * @brief  计算PID控制器输出(位置式)
 * 
 * 该函数根据当前值与目标值之间的误差计算位置式PID控制器的输出。
 * 包含多种改进策略如积分分离、微分滤波、输出限幅等。
 * 
 * @param[in] pid          指向PID控制器结构体的指针
 * @param[in] now_Value    当前值（如：传感器测量值）
 * @param[in] target_Value 目标值（如：设定值）
 * 
 * @return 输出值，根据PID算法计算得出，用于控制系统调整
 */
static float PIDCompute_Position(PIDControllerTypedef *pid, float now_Value, float target_Value)
{
  float output;
  float error;
  
  // 计算误差
  error = target_Value - now_Value;
  pid->error = error;

  // 死区处理
  float proportionalError = error;
  if (fabs(error) < pid->deadBand) 
  {
    proportionalError = 0.0f;
  }
  
  // 积分项计算 (带积分分离)
  if (fabs(error) < pid->integralSeparationThreshold) 
  {
    // 变速积分：误差越小，积分作用越大
    float integralFactor = 1.0f - fabs(error) / pid->integralSeparationThreshold * 0.8f;

    pid->integral += error * integralFactor * pid->sampleTime;
  }
  
  // 积分限幅
  if (pid->integral > pid->integralMax) 
  {
    pid->integral = pid->integralMax;
  }
  else if (pid->integral < pid->integralMin) 
  {
    pid->integral = pid->integralMin;
  }
  
  // 微分项计算 (带滤波)
  float measurement_derivative = (pid->previous_measurement - now_Value) * pid->invSampleTime;
  pid->filteredDerivative = pid->derivativeFilterFactor * measurement_derivative + (1.0f - pid->derivativeFilterFactor) * pid->filteredDerivative;
  
  // 计算PID输出
  output = pid->Kp * proportionalError + 
           pid->Ki * pid->integral + 
           pid->Kd * pid->filteredDerivative;

  // 输出限幅
  if (output > pid->outputMax) 
  {
    output = pid->outputMax;
  } 
  else if (output < pid->outputMin) 
  {
    output = pid->outputMin;
  }
  
  // 保存本次误差
  pid->previous_error = error;
  pid->previous_measurement = now_Value;
  
  return output;
}

/**
 * @brief  计算PID控制器输出(增量式)
 * 
 * 该函数根据当前值与目标值之间的误差计算增量式PID控制器的输出增量。
 * 增量式PID不需要积分项，直接计算输出的增量值。
 * 
 * @param[in] pid          指向PID控制器结构体的指针
 * @param[in] now_Value    当前值（如：传感器测量值）
 * @param[in] target_Value 目标值（如：设定值）
 * 
 * @return 输出增量值，需要与上一次输出累加使用
 */
static float PIDCompute_Delta(PIDControllerTypedef *pid, float now_Value, float target_Value)
{
  float deltaOutput;
  float error;
  
  // 计算误差
  error = target_Value - now_Value;
  
  // 死区处理

  if (fabs(error) < pid->deadBand) 
  {
    error = 0.0f;
  }
  
  // 计算增量式PID各项
  float pPart = pid->Kp * (error - pid->previous_error);
  float iPart = pid->Ki * error * pid->sampleTime;  
  float dPart = pid->Kd * (error - 2.0f * pid->previous_error + pid->prev_prev_error) * pid->invSampleTime;
  
  // 计算输出增量
  deltaOutput = pPart + iPart + dPart;
  
  // 输出增量限幅
  float deltaMax = (pid->outputMax - pid->outputMin) * 0.1f;  // 限制单次变化量为总范围的10%
  if (deltaOutput > deltaMax) 
  {
    deltaOutput = deltaMax;
  } 
  else if (deltaOutput < -deltaMax) 
  {
    deltaOutput = -deltaMax;
  }
  
  // 更新误差记录
  pid->prev_prev_error = pid->previous_error;
  pid->previous_error = error;
  pid->error = error;
  
  return deltaOutput;
}

/**
 * @brief  计算PID控制器输出
 * 
 * 根据PID模式选择位置式或增量式PID计算
 * 
 * @param[in] pid          指向PID控制器结构体的指针
 * @param[in] now_Value    当前值（如：传感器测量值）
 * @param[in] target_Value 目标值（如：设定值）
 * 
 * @return 输出值
 */
float PIDCompute(PIDControllerTypedef *pid, float now_Value, float target_Value)
{
  if (pid == NULL)
    return 0.0f;
      
  pid->targetValue = target_Value;
  
  if (pid->mode == PID_MODE_POSITION) 
  {
    return PIDCompute_Position(pid, now_Value, target_Value);
  } 
  else 
  {
    return PIDCompute_Delta(pid, now_Value, target_Value);
  }
}

/**
 * @brief  设置PID控制器采样时间
 * @param  pid: PID控制器结构体指针
 * @param  sampleTime: 采样时间，单位为秒
 * @note   改变采样时间会自动调整Ki和Kd以保持相同的控制效果
 */
void PID_SetSampleTime(PIDControllerTypedef *pid, float sampleTime)
{
  if (pid == NULL || sampleTime <= 0.0f)
    return;
  
  if (pid->sampleTime != sampleTime) 
  {
    float ratio = sampleTime / pid->sampleTime;
      
    // 调整Ki和Kd以适应新的采样时间
    pid->Ki *= ratio;      // Ki与采样时间成正比
    pid->Kd /= ratio;      // Kd与采样时间成反比
    
    pid->sampleTime = sampleTime;
    pid->invSampleTime = 1.0f / sampleTime;
  }
}

/**
 * @brief  设置PID输出限制
 * @param  pid: PID控制器结构体指针
 * @param  min: 输出最小值
 * @param  max: 输出最大值
 */
void PID_SetOutputLimits(PIDControllerTypedef *pid, float min, float max)
{
  if (pid == NULL || min >= max)
    return;
      
  pid->outputMin = min;
  pid->outputMax = max;
}

/**
 * @brief  设置PID积分项限制
 * @param  pid: PID控制器结构体指针
 * @param  min: 积分项最小值
 * @param  max: 积分项最大值
 */
void PID_SetIntegralLimits(PIDControllerTypedef *pid, float min, float max)
{
  if (pid == NULL || min >= max)
      return;
        
  pid->integralMin = min;
  pid->integralMax = max;
}

/**
 * @brief  设置误差死区
 * @param  pid: PID控制器结构体指针
 * @param  deadBand: 死区值，误差小于该值时不进行控制
 */
void PID_SetDeadBand(PIDControllerTypedef *pid, float deadBand)
{
  if (pid == NULL || deadBand < 0.0f)
      return;
        
  pid->deadBand = deadBand;
}

/**
 * @brief  设置积分分离阈值
 * @param  pid: PID控制器结构体指针
 * @param  threshold: 积分分离阈值，误差大于该值时不计算积分项
 */
void PID_SetIntegralSeparationThreshold(PIDControllerTypedef *pid, float threshold)
{
  if (pid == NULL || threshold < 0.0f)
    return;
        
  pid->integralSeparationThreshold = threshold;
}

/**
 * @brief  设置微分项滤波系数
 * @param  pid: PID控制器结构体指针
 * @param  factor: 滤波系数 (0-1)，1表示不滤波
 */
void PID_SetDerivativeFilterFactor(PIDControllerTypedef *pid, float factor)
{
  if (pid == NULL || factor < 0.0f || factor > 1.0f)
    return;
        
  pid->derivativeFilterFactor = factor;
}

/**
 * @brief  设置PID控制器工作模式
 * @param  pid: PID控制器结构体指针
 * @param  mode: 工作模式 (0: 位置式, 1: 增量式)
 */
void PID_SetMode(PIDControllerTypedef *pid, uint8_t mode)
{
  if (pid == NULL || (mode != PID_MODE_POSITION && mode != PID_MODE_DELTA))
    return;
        
    // 切换模式时重置状态
  if (pid->mode != mode) 
  {
    PID_Reset(pid);
  }
    
  pid->mode = mode;
}

/**
 * @brief  重置PID控制器状态
 * @param  pid: PID控制器结构体指针
 */
void PID_Reset(PIDControllerTypedef *pid)
{
  if (pid == NULL)
    return;
      
  pid->error = 0.0f;
  pid->previous_error = 0.0f;
  pid->prev_prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->derivative = 0.0f;
  pid->filteredDerivative = 0.0f;
  pid->previous_measurement = 0.0f;
}

