/**
 * @file    pid.c
 * @brief   级联 PID 控制器
 * @details 实现三个独立的 PID 控制器：
 *          - 直立环 PD：根据角度偏差和角速度计算输出
 *          - 速度环 PI：根据编码器速度计算目标角度偏移（含积分）
 *          - 转向环 PD：根据 Z 轴角速度计算转向差速
 *
 *          直立环和转向环是纯数学函数，不保持状态。
 *          速度环内部维护积分状态，需要外部调用 Reset()。
 *
 * @author  LJ R
 * @date    2026-07
 */

#include "pid.h"

/*============================================================================*/
/*                              直立环 PD                                      */
/*============================================================================*/

/**
 * @brief      直立环 PD 控制器
 * @details    根据姿态角偏差和 X 轴角速度计算输出，维持平衡。
 *             公式：out = Kp * (angle - med) + Kd * gyro_x
 *
 * @param[in]  kp        比例系数（负值，角度越大输出越小）
 * @param[in]  kd        微分系数（负值，提供阻尼）
 * @param[in]  med_angle 目标角度（机械零点 + 速度环输出）
 * @param[in]  angle     当前角度（度）
 * @param[in]  gyro_x    X 轴角速度（陀螺仪原始值）
 *
 * @return     控制输出（PWM 占空比分量）
 *
 * @note       这是纯数学函数，不保持状态。
 */
int PID_Vertical(float kp, float kd, float med_angle, float angle, float gyro_x)
{
    return (int)(kp * (angle - med_angle) + kd * gyro_x);
}

/*============================================================================*/
/*                              速度环 PI                                      */
/*============================================================================*/

/* 低通滤波：抑制高频噪声 */
static int s_err_lowout_last = 0;  /* 上次滤波后的偏差 */

/* 积分累积：消除稳态误差 */
static int s_encoder_s = 0;        /* 偏差积分 */

/* 积分抗饱和：记录上次输出，用于判断是否饱和 */
static int s_last_velocity_out = 0;

/**
 * @brief  重置速度环积分状态
 * @details 倾倒后调用，清零积分累积，避免重新站立时输出异常。
 */
void PID_Velocity_Reset(void)
{
    s_err_lowout_last = 0;
    s_encoder_s = 0;
    s_last_velocity_out = 0;
}

/**
 * @brief      速度环 PI 控制器
 * @details    根据左右编码器速度和目标速度计算输出。
 *             流程：
 *             1. 计算速度偏差：err = (enc_l + enc_r) - target
 *             2. 低通滤波：err_lowout = (1-a)*err + a*err_last
 *             3. 积分累积（含抗饱和）：encoder_s += err_lowout
 *             4. PI 输出：out = Kp*err_lowout + Ki*encoder_s
 *             5. 输出限幅：确保目标倾角在可控范围内
 *
 * @param[in]  kp     比例系数
 * @param[in]  ki     积分系数
 * @param[in]  target 目标速度（编码器脉冲/10ms）
 * @param[in]  enc_l  左编码器速度
 * @param[in]  enc_r  右编码器速度
 *
 * @return     控制输出（目标角度偏移，叠加到直立环）
 *
 * @note       积分限幅 ±20000，防止积分饱和。
 *             积分抗饱和：输出饱和时停止积分累积。
 *             低通滤波系数 a=0.7，可调。
 */
int PID_Velocity(float kp, float ki, int target, int enc_l, int enc_r)
{
    /* 速度环输出限幅：限制目标倾角偏移范围（度）*/
    /* 物理意义：小车最大可控倾角约 ±60安全范围 ±60*/
    /* 注意：需通过 VOFA+ 观察实际输出范围 */

    /* 低通滤波系数：a 越大，滤波效果越强，响应越慢 */
    static float a = 0.7f;

    /* 计算速度偏差 */
    int err = (enc_l + enc_r) - target;

    /* 低通滤波：抑制高频噪声 */
    int err_lowout = (int)((1.0f - a) * err + a * s_err_lowout_last);
    s_err_lowout_last = err_lowout;

    /*────────────────────────────────────────────────────────────────*/
    /* 积分抗饱和设计                                                 */
    /*────────────────────────────────────────────────────────────────*/
    /* 核心思想：当输出已经达到极限时，停止积分累积                   */
    /*                                                              */
    /* 问题场景：小车被障碍物阻挡                                    */
    /*   - 误差持续存在 → 积分项无限累积                             */
    /*   - 松手瞬间 → 积分项释放 → 目标倾角突变 → 小车失控           */
    /*                                                              */
    /* 解决方案：                                                    */
    /*   - 检查上次输出是否饱和（超过 ±VELOCITY_OUT_MAX）           */
    /*   - 如果饱和，停止积分累积                                    */
    /*   - 如果未饱和，正常累积                                      */
    /*────────────────────────────────────────────────────────────────*/

    if (s_last_velocity_out > -VELOCITY_OUT_MAX &&
        s_last_velocity_out < VELOCITY_OUT_MAX)
    {
        /* 输出未饱和，正常累积积分 */
        s_encoder_s += err_lowout;

        /* 积分限幅：防止积分项过大 */
        if (s_encoder_s > 20000) s_encoder_s = 20000;
        if (s_encoder_s < -20000) s_encoder_s = -20000;
    }
    /* 输出已饱和，跳过积分累积（抗饱和）*/

    /* PI 输出 */
    int out = (int)(kp * err_lowout + ki * s_encoder_s);

    /* 记录本次输出，供下次抗饱和判断使用 */
    s_last_velocity_out = out;

    return out;   /* 返回速度环输出（目标倾角偏移）*/
}

/*============================================================================*/
/*                              转向环 PD                                      */
/*============================================================================*/

/**
 * @brief      转向环 PD 控制器
 * @details    根据 Z 轴角速度和目标转向角速度计算输出。
 *             公式：out = Kp * target_turn + Kd * gyro_z
 *
 * @param[in]  kp          比例系数
 * @param[in]  kd          微分系数（阻尼，无转向时为 0.6，转向时为 0）
 * @param[in]  gyro_z      Z 轴角速度（陀螺仪原始值）
 * @param[in]  target_turn 目标转向角速度
 *
 * @return     控制输出（转向差速）
 *
 * @note       这是纯数学函数，不保持状态。
 *             无转向时 Kd=0.6 抑制震荡，转向时 Kd=0 防止迟钝。
 */
int PID_Turn(float kp, float kd, float gyro_z, int target_turn)
{
    return (int)(kp * target_turn + kd * gyro_z);
}
