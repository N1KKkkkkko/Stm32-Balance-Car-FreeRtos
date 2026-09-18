/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa_debug.c
  * @brief   VOFA+ JustFloat Protocol Debug Module Implementation (TX only)
  *
  * Protocol: JustFloat (Raw Data Mode)
  *   - Frame format: [float0][float1]...[floatN][tail_bytes]
  *   - Tail bytes: 0x00 0x00 0x80 0x7F (float +inf as frame delimiter)
  *   - VOFA+ JustFloat widget auto-splits by tail bytes
  *
  *   蓝牙接收与参数调整由 stm32f1xx_it.c 中断处理，本模块仅负责 TX
  ******************************************************************************
  */
/* USER CODE END Header */

#include "vofa_debug.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
/* FireWater tail bytes: 0x00 0x00 0x80 0x7F (IEEE754 +inf) */
static const uint8_t s_firewater_tail[VOFA_TAIL_SIZE] = {0x00, 0x00, 0x80, 0x7F};

/* Private variables ---------------------------------------------------------*/
Vofa_HandleTypeDef g_vofa = {0};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Copy float channels + tail bytes into TX buffer
  * @retval None
  */
static void Vofa_PrepareTxBuffer(void)
{
    uint8_t *p = g_vofa.tx_buf;
    uint8_t i;

    for (i = 0; i < VOFA_CHANNEL_NUM; i++)
    {
        /* IEEE754 float little-endian copy */
        *p++ = ((uint8_t *)&g_vofa.ch[i])[0];
        *p++ = ((uint8_t *)&g_vofa.ch[i])[1];
        *p++ = ((uint8_t *)&g_vofa.ch[i])[2];
        *p++ = ((uint8_t *)&g_vofa.ch[i])[3];
    }

    for (i = 0; i < VOFA_TAIL_SIZE; i++)
    {
        *p++ = s_firewater_tail[i];
    }
}

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize VOFA debug module
  * @retval None
  */
void Vofa_Init(void)
{
    uint8_t i;

    /* Clear TX buffer */
    for (i = 0; i < VOFA_CHANNEL_NUM; i++)
    {
        g_vofa.ch[i] = 0.0f;
    }
    g_vofa.is_sending = 0;
    g_vofa.tx_cnt = 0;
}

/**
  * @brief  Set a single channel value
  * @param  ch_idx: channel index (0 ~ VOFA_CHANNEL_NUM-1)
  * @param  value: float value
  * @retval None
  */
void Vofa_SetChannel(uint8_t ch_idx, float value)
{
    if (ch_idx < VOFA_CHANNEL_NUM)
    {
        g_vofa.ch[ch_idx] = value;
    }
}

/**
  * @brief  Fill all 10 channels at once (convenience wrapper)
  * @retval None
  */
void Vofa_FillBuffer(float ch0, float ch1, float ch2, float ch3,
                     float ch4, float ch5, float ch6, float ch7,
                     float ch8, float ch9)
{
    g_vofa.ch[0] = ch0;
    g_vofa.ch[1] = ch1;
    g_vofa.ch[2] = ch2;
    g_vofa.ch[3] = ch3;
    g_vofa.ch[4] = ch4;
    g_vofa.ch[5] = ch5;
    g_vofa.ch[6] = ch6;
    g_vofa.ch[7] = ch7;
    g_vofa.ch[8] = ch8;
    g_vofa.ch[9] = ch9;
}

/**
  * @brief  Send one frame via USART3 using DMA
  *         Call this in 10ms control loop for 100Hz refresh rate
  * @retval None
  */
void Vofa_SendFrame(void)
{
    /* Check if previous DMA transfer is complete */
    if (g_vofa.is_sending)
    {
        if (HAL_UART_GetState(&huart3) != HAL_UART_STATE_BUSY_TX)
        {
            g_vofa.is_sending = 0;
        }
        else
        {
            return;  /* Still sending, skip this frame */
        }
    }

    Vofa_PrepareTxBuffer();

    /* Start DMA transmit */
    if (HAL_UART_Transmit_DMA(&huart3, g_vofa.tx_buf, VOFA_TX_BUF_SIZE) == HAL_OK)
    {
        g_vofa.is_sending = 1;
        g_vofa.tx_cnt++;
    }
}

/**
  * @brief  DMA TX complete callback
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        g_vofa.is_sending = 0;
    }
}
