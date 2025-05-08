#include "Cmd_Task.h"
#include <limits.h>
#include <math.h>

// ================= 协议相关常量 =================
#define PROTO_HEAD_2_4G 0xAA
#define PROTO_TAIL_2_4G 0x0F
#define PROTO_HEAD_MIC  0xAA
#define PROTO_TAIL_MIC  0xAA
#define PROTO_CMD_TASK1 0x01
#define PROTO_CMD_TASK2 0x02
#define PROTO_CMD_ERR   0x04
#define CAMERA_DATA_LEN 12
#define GYRO_DATA_LEN   12
#define MIC_MSG_LEN     5

// ===================== 协议相关宏定义 =====================
#define MICROPHONE_MSG_FRAME_HEADER1 0xAA
#define MICROPHONE_MSG_FRAME_HEADER2 0x55
#define MICROPHONE_MSG_FRAME_TAIL1   0x55
#define MICROPHONE_MSG_FRAME_TAIL2   0xAA
#define MICROPHONE_MSG_FRAME_LEN     5

// ================= 全局变量 =================
uint8_t rxBufferFrom2_4G[64] = {0};
uint8_t rxBufferFromCamera[CAMERA_DATA_LEN] = {0};
uint8_t rxBufferFromGyro[GYRO_DATA_LEN] = {0};
uint8_t txBufferFor2_4G[7] = {0};

extern ChassisModeTypeDef chassisMode;
extern uint8_t qrAreaInTask2[2];
extern uint8_t qrNumberInTask1;
extern uint8_t qrNumberInTask2;
extern ChassisTypeDef chassis;

TaskHandle_t Cmd_TaskHandle = NULL;
#define DEBUG_EN  0

// ================== 消息结构体 ===================
typedef struct {
    uint8_t data[3];
} MicrophoneTypeDef;

// ================== 静态函数声明 ==================
static void Handle2_4GNotify(void);
static void HandleCameraNotify(void);
static void HandleGyroNotify(void);
static void HandleCameraProcessingNotify(void);
static void HandleTask2GetQrNotify(void);
static void HandleMicrophoneQueue(void);

// ================== 主任务循环 ===================
void Cmd_Task(void *argument)
{
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, sizeof(rxBufferFrom2_4G));
    vTaskDelay(pdMS_TO_TICKS(10));
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_CAMERA_UART, rxBufferFromCamera, sizeof(rxBufferFromCamera));
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_GYRO_UART, rxBufferFromGyro, sizeof(rxBufferFromGyro));
    while (1)
    {
        uint32_t ulNotifiedValue;
        BaseType_t xResult = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
        #if DEBUG_EN
        VOFA_SendFireWater("%02X %02X %02X %02X %02X %02X %02X\n", rxBufferFrom2_4G[0], rxBufferFrom2_4G[1], rxBufferFrom2_4G[2], rxBufferFrom2_4G[3], rxBufferFrom2_4G[4], rxBufferFrom2_4G[5], rxBufferFrom2_4G[6]);
        #else
        if (xResult == pdPASS)
        {
            if (ulNotifiedValue & NOTIFY_FROM_CMD_2_4G_UART_ISR)
                Handle2_4GNotify();
            if (ulNotifiedValue & NOTIFY_FROM_CMD_CAMERA_UART_ISR)
                HandleCameraNotify();
            if (ulNotifiedValue & NOTIFY_FROM_GYRO_UART_ISR)
                HandleGyroNotify();
            if (ulNotifiedValue & NOTIFY_FROM_CAMERA_PROCESSING)
                HandleCameraProcessingNotify();
            if (ulNotifiedValue & NOTIFY_FROM_TASK2_TO_GET_QR_NUMBER)
                HandleTask2GetQrNotify();
            HandleMicrophoneQueue();
        }
        #endif
        HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, sizeof(rxBufferFrom2_4G));
        HAL_UARTEx_ReceiveToIdle_DMA(CMD_GYRO_UART, rxBufferFromGyro, sizeof(rxBufferFromGyro));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================== 通知处理函数 ===================
static void Handle2_4GNotify(void)
{
  if (rxBufferFrom2_4G[0] == PROTO_HEAD_2_4G && rxBufferFrom2_4G[6] == PROTO_TAIL_2_4G)
  {
    switch (rxBufferFrom2_4G[1])
    {
      case PROTO_CMD_TASK1:
        if (rxBufferFrom2_4G[2] == 0x01 && rxBufferFrom2_4G[3] == 0x01 && rxBufferFrom2_4G[4] == 0x01 && rxBufferFrom2_4G[5] == 0x01)
        {
          chassisMode = CHASSIS_TASK1_READY;
        }
        else if (rxBufferFrom2_4G[2] == 0x02 && rxBufferFrom2_4G[3] == 0x02 && rxBufferFrom2_4G[4] == 0x02 && rxBufferFrom2_4G[5] == 0x02)
        {
          chassisMode = CHASSIS_TASK2_READY;
        }
        // FIXME: 需要结合通讯协议来处理，并且在chassis_Task中根据获取到的area处理获得坐标
        else if (rxBufferFrom2_4G[2] == 0x02 && rxBufferFrom2_4G[3] == 0x02 && rxBufferFrom2_4G[4] == 0x02 && rxBufferFrom2_4G[5] == 0x02)
        {
          qrAreaInTask2[0] = rxBufferFrom2_4G[2];
          qrAreaInTask2[1] = rxBufferFrom2_4G[3];
          xTaskNotify(Chassis_TaskHandle, NOTIFY_QR_AREA_GOTTEN, eSetBits);
        }
        break;
      case PROTO_CMD_ERR:
        // TODO: 错误处理
        break;
      default:
        break;
    }
  }
  HAL_UARTEx_ReceiveToIdle_DMA(CMD_2_4G_UART, rxBufferFrom2_4G, sizeof(rxBufferFrom2_4G));
}

static void HandleCameraNotify(void) // 处理摄像头接收完成通知
{
  uint8_t detectedQrNum = CMD_Camera_CallBack();
  if (detectedQrNum > 0 && detectedQrNum <= 12)
  {
      if (chassisMode == CHASSIS_TASK1_GO_THROUGH)
          qrNumberInTask1 = detectedQrNum;
      else if (chassisMode == CHASSIS_TASK2_GO_THROUGH)
          qrNumberInTask2 = detectedQrNum;
      xTaskNotify(Chassis_TaskHandle, NOTIFY_QR_RESULT_READY, eSetBits);
  }
}

static void HandleGyroNotify(void) // 处理陀螺仪接收完成通知
{
  Cmd_Gyro_CallBack();
  HAL_UARTEx_ReceiveToIdle_DMA(CMD_GYRO_UART, rxBufferFromGyro, sizeof(rxBufferFromGyro));
}

static void HandleCameraProcessingNotify(void) // 处理接收到任务1的开启摄像头通知
{
  HAL_UARTEx_ReceiveToIdle_DMA(CMD_CAMERA_UART, rxBufferFromCamera, sizeof(rxBufferFromCamera));
}

static void HandleTask2GetQrNotify(void) // 处理任务2的二维码获取通知

{
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_CAMERA_UART, rxBufferFromCamera, sizeof(rxBufferFromCamera));
}

static void HandleMicrophoneQueue(void)
{
    MicrophoneTypeDef microphoneMessage;
    if (xQueueReceive(microphoneQueue, &microphoneMessage, 0) == pdPASS)
    {
        for (int i = 0; i < 3; i++)
        {
            SendMessageToMicrophone(microphoneMessage.data[i]);
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }
}

// ================== 协议解析函数 ===================

uint8_t CMD_Camera_CallBack()
{
    uint8_t index = 0;
    for (int i = 0; i < sizeof(rxBufferFromCamera); i++)
    {
        if (rxBufferFromCamera[i] == 'a')
        {
            index = i;
            break;
        }
    }
    uint8_t CameraData[3] = {0};
    if (rxBufferFromCamera[index + 1] == '0')
    {
        for (int i = 0; i < 3; i++)
        {
            CameraData[i] = rxBufferFromCamera[i + 1 + index] - '0';
        }
        uint8_t qrNumber = (CameraData[0] * 100) + (CameraData[1] * 10) + CameraData[2];
        if (qrNumber > 0 && qrNumber <= 12)
            return qrNumber;
        else
            return 0;
    }
    else
    {
        return 0;
    }
}

void Cmd_Gyro_CallBack()
{
    if (rxBufferFromGyro[0] == 0x55 && rxBufferFromGyro[1] == 0x53)
    {
        int16_t rawYaw = (rxBufferFromGyro[7] << 8) | rxBufferFromGyro[6];
        float currentYaw = (float)rawYaw / 32768.0f * 180.0f;
        float continuousYaw = Cmd_Gyro_GetYaw(currentYaw);
        chassis.yaw = continuousYaw;
    }
}

// ===================== 协议与工具函数 =====================
/**
 * @brief  生成麦克风协议帧
 * @param  buf: 输出缓冲区, 长度>=5
 * @param  payload: 有效载荷
 */
static void Microphone_BuildFrame(uint8_t *buf, uint8_t payload)
{
    buf[0] = MICROPHONE_MSG_FRAME_HEADER1;
    buf[1] = MICROPHONE_MSG_FRAME_HEADER2;
    buf[2] = payload;
    buf[3] = MICROPHONE_MSG_FRAME_TAIL1;
    buf[4] = MICROPHONE_MSG_FRAME_TAIL2;
}

/**
 * @brief  发送麦克风协议帧
 * @param  message: 有效载荷
 */
void SendMessageToMicrophone(uint8_t message)
{
    static uint8_t messSend[MICROPHONE_MSG_FRAME_LEN]; // 静态缓冲，减少栈消耗
    Microphone_BuildFrame(messSend, message);
    HAL_UART_Transmit_DMA(CMD_MICROPHONE_UART, messSend, MICROPHONE_MSG_FRAME_LEN);
}

// ===================== 陀螺仪角度连续化 =====================
float Cmd_Gyro_GetYaw(float yaw)
{
    static float preYaw = 0;
    static float fullRotationOffset = 0;
    float dyaw = yaw - preYaw;
    if (fabsf(dyaw) > 180.0f)
    {
        fullRotationOffset += (dyaw > 0) ? -360.0f : 360.0f; // 处理360度跳变
    }
    preYaw = yaw;
    return (fullRotationOffset + yaw);
}