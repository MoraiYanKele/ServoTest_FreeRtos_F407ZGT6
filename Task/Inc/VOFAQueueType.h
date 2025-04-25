#ifndef __VOFAQUEUETYPES_H__
#define __VOFAQUEUETYPES_H__

#include "Vofa_Task.h"

#define VOFA_MAX_CHANNELS 16 // 最大通道数

typedef struct VOFAQueueTypeDef
{
  uint16_t dataLength; // 数据长度
  float data[VOFA_MAX_CHANNELS]; // 数据内容
} VOFAQueueTypeDef;

#endif // __VOFAQUEUETYPES_H__