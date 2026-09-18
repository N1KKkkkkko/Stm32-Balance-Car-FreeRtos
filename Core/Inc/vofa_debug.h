/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa_debug.h
  * @brief   VOFA+ FireWater Protocol Debug Module (TX only)
  *          Supports VOFA+ 1.3.10 real-time waveform display
  *          FireWater protocol: raw float data + tail bytes (0x00 0x00 0x80 0x7F)
  *
  *          蓝牙接收由 remote.c（DMA 空闲中断 + StreamBuffer）处理，本模块仅负责 TX
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __VOFA_DEBUG_H__
#define __VOFA_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart.h"

/* Exported defines ----------------------------------------------------------*/
/* VOFA+ 实时发送总开关：
 *   1 = 启用（ControlTask 每 10ms/100Hz 发送 10 通道数据，含 ch9 稳定性指标）
 *   0 = 关闭（调试期可临时关闭，蓝牙不受影响）
 * USART3 同时承载 VOFA 发送(TX, DMA1_CH2) 与蓝牙接收(RX, DMA1_CH3)，
 * 方向不同互不冲突；100Hz × 44B ≈ 4.4KB/s，115200 波特率余量充足。 */
#define VOFA_ENABLE 1

#define VOFA_CHANNEL_NUM    10      /* Number of float channels to send */
#define VOFA_TAIL_SIZE      4       /* FireWater tail bytes size */
#define VOFA_TX_BUF_SIZE    ((VOFA_CHANNEL_NUM) * sizeof(float) + (VOFA_TAIL_SIZE))

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    /* TX (waveform output) - DMA */
    float ch[VOFA_CHANNEL_NUM];         /* Channel data buffer */
    uint8_t tx_buf[VOFA_TX_BUF_SIZE];   /* Transmit buffer */
    uint8_t is_sending;                 /* TX busy flag */
    uint32_t tx_cnt;                    /* Total transmit counter */
} Vofa_HandleTypeDef;

/* Exported variables --------------------------------------------------------*/
extern Vofa_HandleTypeDef g_vofa;

/* Exported functions --------------------------------------------------------*/
/* Initialization */
void Vofa_Init(void);

/* TX: Waveform output */
void Vofa_SendFrame(void);
void Vofa_SetChannel(uint8_t ch_idx, float value);
void Vofa_FillBuffer(float ch0, float ch1, float ch2, float ch3,
                     float ch4, float ch5, float ch6, float ch7,
                     float ch8, float ch9);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_DEBUG_H__ */
