/**
 * @file    remote.c
 * @brief   蓝牙遥控指令解析模块
 * @details 通过 USART3 + DMA（空闲中断）接收蓝牙数据：
 *          - Remote_Init() 启动 HAL_UARTEx_ReceiveToIdle_DMA() 接收
 *          - 中断回调（HAL_UARTEx_RxEventCallback）只把数据放入流缓冲
 *          - CommTask 调用 Remote_TaskHandler() 在任务上下文解析
 *
 *          支持两类指令：
 *          1. 单字节遥控指令：0x00 停止 / 0x01 前进 / 0x05 后退
 *                            0x03 右转 / 0x07 左转
 *          2. ASCII 参数命令（换行结束）：
 *             - 修改：ZKP=-120 → 设置并回发 ZKP=-120.000 确认
 *             - 查询：ZKP? / ? → 回发当前参数值（供上位机读取）
 *
 * @author  LJ R
 * @date    2026-08
 */

#include "remote.h"
#include "control.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================================================*/
/*                              私有宏                                         */
/*============================================================================*/

#define BT_RX_BUF_SIZE    128   /* USART3 DMA 接收缓冲大小（字节）*/
#define BT_LINE_BUF_SIZE  64    /* ASCII 参数命令行缓冲大小（字节）*/

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/* DMA 接收缓冲（NORMAL 模式，由 HAL_UARTEx_RxEventCallback 搬运）*/
static uint8_t s_bt_rx_buf[BT_RX_BUF_SIZE];

/* 蓝牙数据流缓冲：ISR 写入，CommTask 读取 */
static StreamBufferHandle_t s_bt_stream = NULL;

/* ASCII 参数命令行缓冲 */
static uint8_t s_line_buf[BT_LINE_BUF_SIZE];
static uint8_t s_line_len = 0;

/* 遥控状态标志（0=无指令，1=有指令；CommTask 写，ControlTask 读）*/
static volatile uint8_t s_fore = 0;   /* 前进 */
static volatile uint8_t s_back = 0;   /* 后退 */
static volatile uint8_t s_left = 0;   /* 左转 */
static volatile uint8_t s_right = 0;  /* 右转 */

/*============================================================================*/
/*                              私有函数                                       */
/*============================================================================*/

/**
 * @brief  通过 USART3 回发一行文本（任务上下文）
 * @note   先等 VOFA+ DMA 发送结束再阻塞发送，避免同一串口冲突；
 *         参数命令由人工操作、频率低，短暂阻塞（<10ms）可接受。
 */
static void Remote_SendReply(const char *fmt, ...)
{
    char buf[48];
    va_list ap;
    uint16_t len;
    uint8_t wait = 20;

    while ((HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX) && (wait-- > 0))
    {
        vTaskDelay(pdMS_TO_TICKS(1));   /* 等 VOFA 帧发完（44B @ 115200 ≈ 4ms）*/
    }

    va_start(ap, fmt);
    len = (uint16_t)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if ((len > 0U) && (len < sizeof(buf)))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100);
    }
}

/**
 * @brief  解析一行 ASCII 参数命令（修改 + 查询）
 * @note   在 CommTask 任务上下文调用，可安全使用 atof/strncmp；
 *         修改命令"ZKP=xxx"设置后自动回发新值确认；
 *         查询命令"ZKP?"只回发当前值；"?"回发全部参数。
 */
static void Remote_ParseLine(uint8_t len)
{
    s_line_buf[len] = '\0';

    if (strncmp((char *)s_line_buf, "ZKP", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetVerticalKp((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("ZKP=%.3f\r\n", Control_GetVerticalKp());
    }
    else if (strncmp((char *)s_line_buf, "ZKD", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetVerticalKd((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("ZKD=%.3f\r\n", Control_GetVerticalKd());
    }
    else if (strncmp((char *)s_line_buf, "VKP", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetVelocityKp((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("VKP=%.3f\r\n", Control_GetVelocityKp());
    }
    else if (strncmp((char *)s_line_buf, "VKI", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetVelocityKi((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("VKI=%.3f\r\n", Control_GetVelocityKi());
    }
    else if (strncmp((char *)s_line_buf, "TKP", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetTurnKp((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("TKP=%.3f\r\n", Control_GetTurnKp());
    }
    else if (strncmp((char *)s_line_buf, "TKD", 3) == 0)
    {
        if (s_line_buf[3] == '=') Control_SetTurnKd((float)atof((char *)(s_line_buf + 4)));
        Remote_SendReply("TKD=%.3f\r\n", Control_GetTurnKd());
    }
    else if (strncmp((char *)s_line_buf, "MED_ANGLE", 9) == 0)
    {
        if (s_line_buf[9] == '=') Control_SetMedAngle((float)atof((char *)(s_line_buf + 10)));
        Remote_SendReply("MED_ANGLE=%.3f\r\n", Control_GetMedAngle());
    }
    else if (strcmp((char *)s_line_buf, "?") == 0)
    {
        Remote_SendReply("ZKP=%.3f ZKD=%.3f VKP=%.3f VKI=%.3f TKP=%.3f TKD=%.3f MED_ANGLE=%.3f\r\n",
                         Control_GetVerticalKp(), Control_GetVerticalKd(),
                         Control_GetVelocityKp(), Control_GetVelocityKi(),
                         Control_GetTurnKp(), Control_GetTurnKd(),
                         Control_GetMedAngle());
    }
    else
    {
        Remote_SendReply("ERR\r\n");
    }
}

/**
 * @brief  逐字节处理（任务上下文）
 * @note   单字节指令优先；ASCII 字节累积到换行后按命令行解析
 */
static void Remote_ProcessByte(uint8_t data) 
{
    switch (data)
    {
        case 0x00: case 0x01: case 0x03: case 0x05: case 0x07:
            s_line_len = 0;              /* 清空未完成的行，避免残留 */
            Remote_ParseByte(data);
            break;

        case '\n': case '\r':
            if (s_line_len > 0)
            {
                Remote_ParseLine(s_line_len);
                s_line_len = 0;
            }
            break;

        default:
            if (s_line_len < BT_LINE_BUF_SIZE - 1)
            {
                s_line_buf[s_line_len++] = data;
            }
            else
            {
                s_line_len = 0;          /* 行超长，丢弃重来 */
            }
            break;
    }
}

/*============================================================================*/
/*                              指令解析                                       */
/*============================================================================*/

/**
 * @brief      解析蓝牙单字节指令
 * @details    由 Remote_ProcessByte() 在任务上下文调用，更新状态标志。
 *
 * @param[in]  data 蓝牙收到的单字节指令
 *
 * @note       指令协议：
 *             - 0x00：停止（清零所有标志）
 *             - 0x01：前进
 *             - 0x05：后退
 *             - 0x03：右转
 *             - 0x07：左转
 */
void Remote_ParseByte(uint8_t data)
{
    switch (data)
    {
        case 0x00: s_fore = 0; s_back = 0; s_left = 0; s_right = 0; break;
        case 0x01: s_fore = 1; s_back = 0; s_left = 0; s_right = 0; break;
        case 0x05: s_fore = 0; s_back = 1; s_left = 0; s_right = 0; break;
        case 0x03: s_fore = 0; s_back = 0; s_left = 0; s_right = 1; break;
        case 0x07: s_fore = 0; s_back = 0; s_left = 1; s_right = 0; break;
        default:   s_fore = 0; s_back = 0; s_left = 0; s_right = 0; break;
    }
}

/*============================================================================*/
/*                              初始化与任务体                                 */
/*============================================================================*/

/**
 * @brief  初始化蓝牙模块
 * @details 创建流缓冲并启动 USART3 DMA 空闲接收。
 *          DMA 通道（DMA1_Channel3）已在 CubeMX/usart.c 中配置好。
 *          必须在 FreeRTOS 调度器启动后（CommTask 内）调用，
 *          因为创建流缓冲需要 FreeRTOS 堆。
 */
void Remote_Init(void)
{
    if (s_bt_stream == NULL)
    {
        s_bt_stream = xStreamBufferCreate(BT_RX_BUF_SIZE * 2, 1);
        configASSERT(s_bt_stream != NULL);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_bt_rx_buf, BT_RX_BUF_SIZE);
}

/**
 * @brief  蓝牙数据处理任务体（由 CommTask 循环调用）
 * @details 阻塞等待流缓冲数据，逐字节解析。
 *          解析（含 atof/strncmp）在任务上下文完成，不再占用中断。
 */
void Remote_TaskHandler(void)
{
    uint8_t chunk[BT_RX_BUF_SIZE];
    size_t n;
    size_t i;

    n = xStreamBufferReceive(s_bt_stream, chunk, sizeof(chunk), portMAX_DELAY);
    for (i = 0; i < n; i++)
    {
        Remote_ProcessByte(chunk[i]);
    }
}

/*============================================================================*/
/*                              状态查询                                       */
/*============================================================================*/

/**
 * @brief  查询前进状态
 * @return 1=前进，0=无前进指令
 */
uint8_t Remote_IsForward(void) { return s_fore; }

/**
 * @brief  查询后退状态
 * @return 1=后退，0=无后退指令
 */
uint8_t Remote_IsBackward(void) { return s_back; }

/**
 * @brief  查询左转状态
 * @return 1=左转，0=无左转指令
 */
uint8_t Remote_IsLeft(void) { return s_left; }

/**
 * @brief  查询右转状态
 * @return 1=右转，0=无右转指令
 */
uint8_t Remote_IsRight(void) { return s_right; }

/*============================================================================*/
/*                              HAL 回调（中断上下文）                          */
/*============================================================================*/

/**
 * @brief  USART3 DMA 空闲接收事件回调（中断上下文）
 * @note   只做搬运 + 重启接收，不做任何解析；
 *         使用 FromISR 版 API 写入流缓冲。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART3)
    {
        if ((s_bt_stream != NULL) && (Size > 0))
        {
            xStreamBufferSendFromISR(s_bt_stream, s_bt_rx_buf, Size,
                                     &xHigherPriorityTaskWoken);
        }
        /* 重启下一次 DMA 空闲接收（DMA NORMAL 模式，必须重新启动）*/
        HAL_UARTEx_ReceiveToIdle_DMA(huart, s_bt_rx_buf, BT_RX_BUF_SIZE);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  USART3 错误回调（中断上下文）
 * @note   串口错误（overrun/framing/noise）会中断接收链，
 *         这里清理标志并重启 DMA 接收，避免蓝牙永久失效。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
        {
            HAL_UART_AbortReceive(huart);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, s_bt_rx_buf, BT_RX_BUF_SIZE);
    }
}
