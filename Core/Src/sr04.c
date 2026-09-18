/**
 * @file    sr04.c
 * @brief   HC-SR04 超声波测距模块
 * @details 模块化设计：
 *          - 触发：PA3 输出 10us 低脉冲（SR04_Trigger）
 *          - 回波：PA9 输入捕获（TIM1_CH2），复用 TIM1 计数时钟
 *          - 测距：上升沿记录起点 → 下降沿计算高电平时间 → 换算距离（cm）
 *
 *          时基说明（为什么用 TIM1 且需要溢出计数）：
 *          TIM1 的 ARR=7199（100us 周期）被 PWM 通道 CH1/CH4 占用，不能修改；
 *          HC-SR04 回波最长约 38ms，远大于 100us，因此：
 *          - 测量期间才使能 TIM1 更新中断，在 HAL_TIM_PeriodElapsedCallback
 *            （freertos.c）中累计溢出次数，并经 SR04_OnTimerUpdate 转发到本模块；
 *          - 回波高电平时间 = 溢出次数*(ARR+1) + 下降沿捕获值 - 上升沿捕获值。
 *          溢出中断只在测量期间开启，平时不产生额外中断负载。
 *
 * @author  LJ R
 * @date    2026-08
 */

#include "sr04.h"
#include "tim.h"

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/* 测量状态（ISR 与任务共享，volatile 防止优化）*/
static volatile uint8_t  s_measuring;    /* 1 = 正在等待回波下降沿 */
static volatile uint32_t s_ovf_cnt;      /* 测量期间 TIM1 溢出次数 */
static volatile uint16_t s_cap_rising;   /* 上升沿捕获的计数器值 */

/* 最近一次测量距离（cm）；-1 = 无有效回波（兼容 sensor.c 的 extern 读取）*/
float distance = -1.0f;

/*============================================================================*/
/*                              私有函数                                       */
/*============================================================================*/

/**
 * @brief  微秒级延时（简单空循环忙等）
 * @note   FreeRTOS 下无法用 osDelay 产生 us 级延时（最小粒度 1ms），
 *         只能忙等；但每次仅 ~20us，且忙等期间高优先级任务仍会被
 *         1ms SysTick 抢占，不影响控制环。系数 9 与原工程 RCCdelay_us
 *         （us*72/8）一致，按 72MHz、每迭代约 6~9 周期校准。
 */
static void SR04_DelayUs(uint32_t us)
{
  __IO uint32_t delay = us * 9U;
  while (delay--)
  {
    __NOP();
  }
}

/**
 * @brief  复位测量状态：关闭溢出计数中断、切回上升沿捕获
 */
static void SR04_Rearm(void)
{
  s_measuring = 0;
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
}

/*============================================================================*/
/*                              对外接口                                       */
/*============================================================================*/

/**
 * @brief  初始化超声波模块：启动 TIM1_CH2 输入捕获（上升沿）
 * @note   在 SR04_Task 里调用（调度器启动后），与 Remote_Init 模式一致
 */
void SR04_Init(void)
{
  distance = -1.0f;
  SR04_Rearm();
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief  发送 10us 触发脉冲（Trig=PA3）
 */
void SR04_Trigger(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  SR04_DelayUs(20);   /* 脉宽按 >=10us 规格留余量（HC-SR04 触发要求 10us）*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

/**
 * @brief  是否正在测量（等待回波下降沿）
 */
uint8_t SR04_IsMeasuring(void)
{
  return s_measuring;
}

/**
 * @brief  测量超时处理：无回波时调用，清除挂起的测量状态
 */
void SR04_Abort(void)
{
  if (s_measuring)
  {
    SR04_Rearm();
    distance = -1.0f;   /* 视为无障碍 */
  }
}

/**
 * @brief  获取最近一次测量距离
 * @return 距离（cm）；-1 表示无有效回波（无障碍/超时）
 */
float SR04_GetDistanceCm(void)
{
  return distance;
}

/*============================================================================*/
/*                            TIM1 中断回调（HAL）                             */
/*============================================================================*/

/**
 * @brief  TIM1 更新中断转发入口（由 freertos.c 的
 *         HAL_TIM_PeriodElapsedCallback 的 TIM1 分支调用）
 * @note   ISR 上下文：只做溢出计数，不做其他处理
 */
void SR04_OnTimerUpdate(void)
{
  if (s_measuring)
  {
    s_ovf_cnt++;
  }
}

/**
 * @brief  TIM1_CH2 输入捕获回调（HAL 库弱函数，本模块实现）
 * @note   ISR 上下文：只记录捕获值、切换极性、换算距离，不调用 RTOS API
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
  {
    /* 用模块状态判断当前等待的边沿（该 HAL 版本无 GET_CAPTUREPOLARITY 宏）：
     * s_measuring==0 → 等待上升沿；==1 → 等待下降沿 */
    if (!s_measuring)
    {
      /* 上升沿：回波开始 */
      s_cap_rising = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
      s_ovf_cnt = 0;
      s_measuring = 1;

      /* 切换为下降沿捕获，并开启更新中断用于溢出计数 */
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);   /* 清掉历史溢出标志，避免开始即多计一次 */
      __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    }
    else
    {
      /* 下降沿：回波结束，计算高电平时间（72MHz tick）*/
      uint32_t period = htim->Init.Period + 1U;                        /* 7200 */
      uint32_t cap    = (uint32_t)HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
      uint32_t ticks  = s_ovf_cnt * period;

      if (cap >= s_cap_rising)
      {
        ticks += (cap - (uint32_t)s_cap_rising);
      }
      else
      {
        ticks += period - ((uint32_t)s_cap_rising - cap);   /* 跨越了一次溢出 */
      }

      /* 复位状态、关闭溢出中断、切回上升沿 */
      SR04_Rearm();

      /* 换算距离：72MHz → us → cm（HC-SR04：58us 对应 1cm）*/
      if (ticks > 0U)
      {
        distance = ((float)ticks / 72.0f) / 58.0f;
      }
      else
      {
        distance = -1.0f;
      }
    }
  }
}
