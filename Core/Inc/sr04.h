#ifndef _SR04_H
#define _SR04_H

#include "stm32f1xx_hal.h"

/**
 * @file    sr04.h
 * @brief   HC-SR04 超声波测距模块（模块化接口）
 * @note    引脚：Trig=PA3（GPIO 输出），Echo=PA9（TIM1_CH2 输入捕获）
 */

/* 超声波模块总开关：
 *   1 = 启用（触发测距 + 避障）
 *   0 = 禁用（硬件模块故障时临时关闭）
 * 禁用后：SR04_Task 不创建、不触发测距，Control_SetObstacle 不再被调用，
 *         避障自动失效（s_obstacle_cm 恒为 -1），其余模块不受影响。
 * 恢复方法：硬件修好后把本宏改回 1 即可，无需改动其他文件。 */
#define SR04_ENABLE 0

/* 初始化：启动 TIM1_CH2 输入捕获（在 SR04_Task 内调用）*/
void SR04_Init(void);

/* 发送 10us 触发脉冲（Trig=PA3）*/
void SR04_Trigger(void);

/* 是否正在等待回波（1=测量中）*/
uint8_t SR04_IsMeasuring(void);

/* 测量超时处理：无回波时调用，清除挂起状态并置 distance=-1 */
void SR04_Abort(void);

/* 获取最近一次测量距离（cm）；-1 = 无有效回波（无障碍/超时）*/
float SR04_GetDistanceCm(void);

/* TIM1 更新中断转发入口（由 freertos.c 的 HAL_TIM_PeriodElapsedCallback 调用）*/
void SR04_OnTimerUpdate(void);

#endif
