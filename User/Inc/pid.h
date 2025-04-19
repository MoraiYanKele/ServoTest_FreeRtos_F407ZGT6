#ifndef __PID_H__
#define __PID_H__

#include "main.h"

// PID控制器模式定义
#define PID_MODE_POSITION 0  // 位置式PID
#define PID_MODE_DELTA    1  // 增量式PID

typedef struct 
{
    // PID参数
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    
    // 控制目标与状态
    float targetValue;   // 目标值
    float error;         // 当前误差
    float previous_error; // 上一次误差
    float prev_prev_error; // 上上次误差(用于增量式PID)
    float integral;      // 积分项
    float derivative;    // 微分项
    
    // 输出限制
    float outputMax;     // 输出上限
    float outputMin;     // 输出下限
    
    // 积分限制
    float integralMax;   // 积分上限
    float integralMin;   // 积分下限
    
    // 死区设置
    float deadBand;      // 误差死区，小于该值不进行控制
    
    // 积分分离阈值
    float integralSeparationThreshold; // 误差大于该阈值时，不计算积分项
    
    // 微分滤波系数 (0-1), 1表示不滤波
    float derivativeFilterFactor;
    
    // 控制器工作模式
    uint8_t mode;        // 0: 位置式PID, 1: 增量式PID
    
    // 滤波后的微分项(仅内部使用)
    float filteredDerivative;

    float sampleTime;        // 采样时间，单位为秒
    float invSampleTime;     // 采样时间的倒数，用于优化计算

    float previous_measurement; // 添加这个新成员来存储上一次的测量值
    
} PIDControllerTypedef;
  
// 函数声明
void PID_Init(PIDControllerTypedef *pid, float Kp, float Ki, float Kd);
float PIDCompute(PIDControllerTypedef *pid, float now_Value, float target_Value);
void PID_SetOutputLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetIntegralLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetDeadBand(PIDControllerTypedef *pid, float deadBand);
void PID_SetIntegralSeparationThreshold(PIDControllerTypedef *pid, float threshold);
void PID_SetDerivativeFilterFactor(PIDControllerTypedef *pid, float factor);
void PID_SetMode(PIDControllerTypedef *pid, uint8_t mode);
void PID_Reset(PIDControllerTypedef *pid);
void PID_SetSampleTime(PIDControllerTypedef *pid, float sampleTime);


#endif