/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS 任务实现
  * @details 实现四个任务：
  *          - ControlTask：控制环任务（10ms 硬件节拍，TIM3 ISR 驱动）
  *          - CommTask：通信任务（蓝牙 DMA 空闲接收 + 任务化解析）
  *          - DisplayTask：显示任务（OLED 刷新，200ms 周期）
  *          - defaultTask：默认任务（空闲，挂起）
  *
  *          任务优先级（从高到低）：
  *          - ControlTask：Realtime（48）
  *          - CommTask：AboveNormal（32）
  *          - DisplayTask：BelowNormal（8）
  *          - defaultTask：Normal（24）
  *
  *          控制周期：
  *          - 10ms 硬件定时器（TIM3 更新中断 → osThreadFlagsSet → 唤醒 ControlTask）
  *
  * @author  LJ R
  * @date    2026-07
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor.h"
#include "control.h"
#include "remote.h"
#include "oled.h"
#include "sr04.h"
#include "usart.h"
#include "tim.h"
#include "vofa_debug.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/* 控制周期标志位（bit0）*/
#define CONTROL_FLAG 0x01U

/* OLED 显示缓冲 */
static uint8_t s_disp_buf[24];

/* 超声波测距任务句柄与属性（CubeMX 未配置，手动创建，放 USER CODE 避免被重新生成覆盖）*/
osThreadId_t SR04Handle;
const osThreadAttr_t SR04_attributes = {
  .name = "SR04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Comm */
osThreadId_t CommHandle;
const osThreadAttr_t Comm_attributes = {
  .name = "Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SR04_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ControlTask(void *argument);
void CommTask(void *argument);
void DisplayTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Control */
  ControlHandle = osThreadNew(ControlTask, NULL, &Control_attributes);

  /* creation of Comm */
  CommHandle = osThreadNew(CommTask, NULL, &Comm_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(DisplayTask, NULL, &Display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
#if SR04_ENABLE
  /* 超声波测距任务：约 60ms 周期触发 HC-SR04 并更新障碍距离 */
  SR04Handle = osThreadNew(SR04_Task, NULL, &SR04_attributes);
#endif /* SR04_ENABLE */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务（空闲任务）
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* defaultTask 无业务逻辑，永久挂起释放 CPU */
  osDelay(osWaitForever);
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief  控制环任务（Realtime 优先级）
* @details 执行流程：
*          1. 启动 10ms 硬件定时器（TIM3 更新中断）
*          2. 阻塞等待 TIM3 中断发来的线程标志
*          3. 执行传感器更新和 PID 控制
*
*          控制周期：10ms（TIM3 硬件定时器 ISR → osThreadFlagsSet，CMSIS-RTOS2 API）
*          优先级：Realtime（48），最高优先级，ISR 置位后立即抢占执行
*
* @note   为什么用硬件定时器而不是软件定时器：软件定时器回调要经过 Timer
*          服务任务（优先级 40）转发，再唤醒本任务，存在调度抖动；TIM3 更新
*          中断直接 osThreadFlagsSet，ISR 给出精确 10ms 节拍。
*          TIM3 原用于超声波回波计时，现改作控制节拍；HC-SR04 测距改用 TIM1_CH2（见 sr04.c）。
*
* @param[in] argument 任务参数（未使用）
*/
/* USER CODE END Header_ControlTask */
__weak void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  /* 1. 启动 10ms 硬件定时器（TIM3），更新中断中 osThreadFlagsSet 唤醒本任务
   *    Sensor_Init() 已在 main.c 调度器启动前完成 */
  HAL_TIM_Base_Start_IT(&htim3);

  /* 2. 控制环主循环 */
  for(;;)
  {
    /* 阻塞等待 TIM3 中断发来的标志（不占用 CPU） */
    osThreadFlagsWait(CONTROL_FLAG, osFlagsWaitAny, osWaitForever);

    /* 读取传感器 & 执行 PID 控制 */
    Sensor_Update();
    Control_Run();

    /* VOFA+ 数据发送（10 个通道，参照 VOFA 项目）*/
    /* 通道分配：
     *   ch0: 横滚角（度）
     *   ch1: X 轴角速度（原始值）
     *   ch2: Y 轴角速度（原始值）
     *   ch3: 左编码器速度
     *   ch4: 右编码器速度
     *   ch5: 直立环输出（PWM）
     *   ch6: 速度环输出（目标倾角偏移）
     *   ch7: 左电机 PWM
     *   ch8: 右电机 PWM
     *   ch9: Roll 移动平均差（度）
     */
#if VOFA_ENABLE
    Vofa_FillBuffer(
        Sensor_GetRoll(),                     /* ch0: 横滚角 */
        (float)Sensor_GetGyroX(),             /* ch1: X 轴角速度 */
        (float)Sensor_GetGyroY(),             /* ch2: Y 轴角速度 */
        (float)Sensor_GetEncoderLeft(),       /* ch3: 左编码器 */
        (float)Sensor_GetEncoderRight(),      /* ch4: 右编码器 */
        (float)Control_GetVerticalOut(),      /* ch5: 直立环输出 */
        (float)Control_GetVelocityOut(),      /* ch6: 速度环输出 */
        (float)Control_GetMoto1(),            /* ch7: 左电机 PWM */
        (float)Control_GetMoto2(),            /* ch8: 右电机 PWM */
        Sensor_GetRollDiff()                  /* ch9: Roll 移动平均差 */
    );
    Vofa_SendFrame();  /* 通过 USART3 发送（DMA） */
#endif
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_CommTask */
/**
* @brief  通信任务（AboveNormal 优先级）
* @details 初始化蓝牙 DMA 空闲接收，
*          在任务上下文轮询流缓冲并解析指令。
*
* @param[in] argument 任务参数（未使用）
*/
/* USER CODE END Header_CommTask */
__weak void CommTask(void *argument)
{
  /* USER CODE BEGIN CommTask */
  /* 1. 初始化蓝牙：创建流缓冲 + 启动 USART3 DMA 空闲接收 */
  Remote_Init();

  /* 2. 循环处理蓝牙数据（阻塞等待流缓冲，解析在任务上下文完成）*/
  for(;;)
  {
    Remote_TaskHandler();
  }
  /* USER CODE END CommTask */
}

/* USER CODE BEGIN Header_DisplayTask */
/**
* @brief  显示任务（BelowNormal 优先级）
* @details 每 200ms 刷新一次 OLED：
*          - 第 0 行：左编码器速度
*          - 第 2 行：右编码器速度
*          - 第 4 行：横滚角
*          - 第 6 行：左电机 PWM
*
*          刷新周期：200ms
*          优先级：BelowNormal（8），最低优先级，避免抢占控制任务
*
* @param[in] argument 任务参数（未使用）
*/
/* USER CODE END Header_DisplayTask */
__weak void DisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */
  for(;;)
  {
    /* 显示编码器速度 */
    sprintf((char *)s_disp_buf, "L:%4d  ", Sensor_GetEncoderLeft());
    OLED_ShowString(0, 0, s_disp_buf, 16);
    sprintf((char *)s_disp_buf, "R:%4d  ", Sensor_GetEncoderRight());
    OLED_ShowString(0, 2, s_disp_buf, 16);

    /* 显示姿态角 */
    sprintf((char *)s_disp_buf, "Roll:%.1f  ", Sensor_GetRoll());
    OLED_ShowString(0, 4, s_disp_buf, 16);

    /* 显示控制输出 */
    sprintf((char *)s_disp_buf, "M1:%4d  ", Control_GetMoto1());
    OLED_ShowString(0, 6, s_disp_buf, 12);

    /* 200ms 刷新一次，避免频繁刷屏 */
    osDelay(200);
  }
  /* USER CODE END DisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if SR04_ENABLE
/**
 * @brief  超声波避障任务
 * @details 循环：触发 HC-SR04 → 等待回波（含超时保护）→ 把距离交给控制层
 *          周期约 60ms（满足 HC-SR04 两次触发间隔 >= 60ms 的要求）。
 *          模块职责划分：本任务只做"测距编排"；测量逻辑在 sr04.c，
 *          避障决策在 control.c（Control_SetObstacle）。
 * @note   由 sr04.h 的 SR04_ENABLE 控制：=0 时不创建本任务（当前禁用）。
 */
void SR04_Task(void *argument)
{
  SR04_Init();   /* 启动 TIM1_CH2 输入捕获 */

  for (;;)
  {
    SR04_Trigger();                        /* 10us 触发脉冲（PA3）*/

    osDelay(60);                           /* 等待回波完成（最长约 38ms）*/

    if (SR04_IsMeasuring())                /* 60ms 后仍在测量 = 无回波 */
    {
      SR04_Abort();
    }

    Control_SetObstacle(SR04_GetDistanceCm());   /* 交给控制层做避障 */
  }
}
#endif /* SR04_ENABLE */

/**
 * @brief  TIM3 更新中断回调（10ms 控制节拍）
 * @note   ISR 上下文：只做标志通知（osThreadFlagsSet 底层走
 *          xTaskNotifyFromISR），不执行任何业务逻辑。
 * @param[in] htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    osThreadFlagsSet(ControlHandle, CONTROL_FLAG);
  }
  else if (htim->Instance == TIM1)
  {
    /* 超声波回波测量期间的 TIM1 溢出计数（仅测量时使能更新中断，见 sr04.c）*/
    SR04_OnTimerUpdate();
  }
}

/**
 * @brief  栈溢出钩子函数
 * @details 当 FreeRTOS 检测到任务栈溢出时调用（死循环便于调试）
 *
 * @param[in] xTask      任务句柄
 * @param[in] pcTaskName 任务名称
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  for(;;);
}
/* USER CODE END Application */  
