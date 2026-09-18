/**
 * @file    sensor.c
 * @brief   传感器数据采集模块
 * @details 封装 MPU6050 和编码器的数据读取，提供统一接口：
 *          - 初始化：Sensor_Init()
 *          - 更新：Sensor_Update()（在控制任务中调用）
 *          - 访问：Sensor_GetRoll()、Sensor_GetEncoderLeft() 等
 *
 *          传感器数据通过私有静态变量缓存，避免直接暴露全局变量。
 *
 * @author  LJ R
 * @date    2026-07
 */

#include "sensor.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

static int s_encoder_left = 0;   /* 左编码器速度（脉冲数/10ms）*/
static int s_encoder_right = 0;  /* 右编码器速度（脉冲数/10ms）*/
static float s_roll = 0;         /* 横滚角（度）*/
static short s_gyrox = 0;        /* X 轴角速度（陀螺仪原始值）*/
static short s_gyroy = 0;        /* Y 轴角速度（用于 VOFA+ 调试）*/
static short s_gyroz = 0;        /* Z 轴角速度 */
static float s_distance = 0;     /* 超声波距离（cm）*/

/* Roll 移动平均差计算（用于 VOFA+ 调试）*/
#define ROLL_HISTORY_SIZE 5
static float s_roll_history[ROLL_HISTORY_SIZE] = {0};  /* Roll 历史缓冲区 */
static uint8_t s_roll_history_index = 0;               /* 缓冲区索引 */
static float s_roll_diff = 0;                          /* Roll 移动平均差 */

extern TIM_HandleTypeDef htim2, htim4;
extern float distance;  /* 超声波距离，由 sr04.c 更新 */

/*============================================================================*/
/*                              初始化                                         */
/*============================================================================*/

/**
 * @brief  初始化传感器硬件
 * @details 调用顺序：
 *          1. MPU6050 初始化（I2C、陀螺仪零点校准）
 *          2. DMP 固件加载（姿态解算）
 *          3. 编码器定时器启动
 *
 * @note   必须在 FreeRTOS 调度器启动前调用，避免任务切换导致初始化失败
 */
void Sensor_Init(void)
{
    MPU_Init();         /* 初始化 MPU6050（I2C、量程配置）*/
    mpu_dmp_init();     /* 加载 DMP 固件（耗时约 100ms）*/
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* 左编码器 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* 右编码器 */
}

/*============================================================================*/
/*                              数据更新                                       */
/*============================================================================*/

/**
 * @brief  更新传感器数据（每 10ms 调用一次）
 * @details 从硬件读取最新数据并存入私有变量：
 *          - 编码器：TIM2/TIM4 计数器
 *          - 姿态角：DMP 解算（roll 为主，pitch/yaw 丢弃）
 *          - 角速度：陀螺仪原始值
 *          - 超声波：从 sr04.c 的全局变量同步
 *
 * @note   DMP 的 mpu_dmp_get_data() 必须同时取 pitch/roll/yaw，
 *         不能只取 roll，否则 FIFO 会错位
 */
void Sensor_Update(void)
{
    float pitch_unused, yaw_unused;

    /* 读取编码器速度并清零计数器 */
    s_encoder_left = Read_Speed(&htim2);
    s_encoder_right = -Read_Speed(&htim4);  /* 右轮反向安装 */

    /* 读取姿态角（DMP 解算）*/
    mpu_dmp_get_data(&pitch_unused, &s_roll, &yaw_unused);

    /* 读取角速度（原始值）*/
    MPU_Get_Gyroscope(&s_gyrox, &s_gyroy, &s_gyroz);

    /* 同步超声波距离 */
    s_distance = distance;

    /* 计算 Roll 移动平均差（当前值与 N 个周期前的差值）*/
    s_roll_diff = s_roll - s_roll_history[s_roll_history_index];
    s_roll_history[s_roll_history_index] = s_roll;
    s_roll_history_index = (s_roll_history_index + 1) % ROLL_HISTORY_SIZE;
}

/*============================================================================*/
/*                              访问接口                                       */
/*============================================================================*/

float Sensor_GetRoll(void)         { return s_roll; }
float Sensor_GetRollDiff(void)     { return s_roll_diff; }
short Sensor_GetGyroX(void)        { return s_gyrox; }
short Sensor_GetGyroY(void)        { return s_gyroy; }
short Sensor_GetGyroZ(void)        { return s_gyroz; }
int   Sensor_GetEncoderLeft(void)  { return s_encoder_left; }
int   Sensor_GetEncoderRight(void) { return s_encoder_right; }
float Sensor_GetDistance(void)     { return s_distance; }
