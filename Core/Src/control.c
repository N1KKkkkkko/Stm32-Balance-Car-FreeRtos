/**
 * @file    control.c
 * @brief   平衡小车控制模块
 * @details 实现级联 PID 控制算法：
 *          - 直立环 PD：根据姿态角偏差和角速度计算输出
 *          - 速度环 PI：根据编码器速度计算目标角度偏移
 *          - 转向环 PD：根据 Z 轴角速度控制转向
 *
 *          控制周期：10ms（由 FreeRTOS 软件定时器驱动）
 *          输出：左右电机 PWM 占空比
 *
 * @author  LJ R
 * @date    2026-07
 */

#include "control.h"
#include "sensor.h"
#include "remote.h"
#include "pid.h"
#include "motor.h"

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/* PID 控制输出 */
static int s_vertical_out = 0;   /* 直立环输出 */
static int s_velocity_out = 0;   /* 速度环输出（叠加到目标角度）*/
static int s_turn_out = 0;       /* 转向环输出 */
static int s_moto1 = 0;          /* 左电机 PWM */
static int s_moto2 = 0;          /* 右电机 PWM */

/* 目标速度和转向（用于 VOFA+ 调试）*/
static int s_target_speed = 0;   /* 目标速度（编码器脉冲/10ms）*/
static int s_target_turn = 0;    /* 目标转向角速度 */

/* 机械零点：小车平衡时的自然倾斜角（需根据重心位置调试）*/
static float s_med_angle = -2.0f;

/* 倾倒保护标志：偏差超过 60 度时置 1 */
static uint8_t s_stopped = 0;

/* 超声波障碍距离（cm），-1 = 无有效回波（无障碍）*/
static float s_obstacle_cm = -1.0f;

/*============================================================================*/
/*                              PID 参数                                       */
/*============================================================================*/

/* 直立环 PD 参数（关键参数，直接影响平衡稳定性）*/
static float s_vertical_kp = -120.0f;  /* 比例系数：角度偏差的响应强度 */
static float s_vertical_kd = -0.80f;    /* 微分系数：角速度的阻尼效果 */

/* 速度环 PI 参数（控制前进后退速度）*/
static float s_velocity_kp = -1.1f;    /* 比例系数 */
static float s_velocity_ki = -0.0055f; /* 积分系数：消除稳态误差 */

/* 转向环 PD 参数（控制转向）*/
static float s_turn_kp = 10.0f;        /* 比例系数 */
static float s_turn_kd = 0.1f;         /* 微分系数：转向阻尼 */

/*============================================================================*/
/*                              速度限制宏                                     */
/*============================================================================*/

#define SPEED_Y 20   /* 前进后退速度上限（编码器脉冲/10ms）*/
#define SPEED_Z 150  /* 转向速度上限 */

/* 超声波避障阈值（cm），可按实际场地调整 */
#define OBSTACLE_SLOW_CM 40.0f   /* 进入避障距离：半速后退 */
#define OBSTACLE_STOP_CM 20.0f   /* 紧急距离：全力后退 */

/**
 * @brief  控制环主函数（每 10ms 执行一次）
 * @details 执行流程：
 *          1. 读取传感器数据（编码器、姿态角、角速度）
 *          2. 处理遥控指令（渐变加速/减速）
 *          3. 执行级联 PID 计算
 *          4. 输出电机 PWM
 *
 *          级联结构：
 *          速度环 PI → 目标角度偏移
 *          直立环 PD + 目标角度 → 垂直输出
 *          转向环 PD → 左右差速
 *
 * @note   必须以固定周期（10ms）调用，否则 PID 参数会失效
 */
void Control_Run(void)
{
    int enc_l, enc_r;
    float roll;
    short gyrox, gyroz;

    /*--------- 1. 读取传感器数据 ---------*/
    enc_l = Sensor_GetEncoderLeft();
    enc_r = Sensor_GetEncoderRight();
    roll = Sensor_GetRoll();
    gyrox = Sensor_GetGyroX();
    gyroz = Sensor_GetGyroZ();

    /*--------- 2. 处理遥控指令（渐变加速）---------*/
    /* 松手归零：无前进/后退指令时速度归零 */
	
    if (!Remote_IsForward() && !Remote_IsBackward())
        s_target_speed = 0;

    /* 物理方向约定（本机接线实测）：s_target_speed 为正时小车实际是后退。
     * 因此"前进"指令给负目标速度、"后退"给正目标速度。
     * 只交换参考符号，不改动控制环/电机/编码器，不影响平衡。 */
    if (Remote_IsForward()) s_target_speed--;    /* 前进 */
    if (Remote_IsBackward()) s_target_speed++;   /* 后退 */

    /* 速度限幅：防止过冲 */
    if (s_target_speed > SPEED_Y) s_target_speed = SPEED_Y;
    if (s_target_speed < -SPEED_Y) s_target_speed = -SPEED_Y;

    /* 转向控制 */
    if (!Remote_IsLeft() && !Remote_IsRight())
        s_target_turn = 0;
    if (Remote_IsLeft()) s_target_turn += 30;
    if (Remote_IsRight()) s_target_turn -= 30;
    if (s_target_turn > SPEED_Z) s_target_turn = SPEED_Z;
    if (s_target_turn < -SPEED_Z) s_target_turn = -SPEED_Z;

    /* 转向阻尼：无转向时增加阻尼，抑制高频震荡 */
    if (!Remote_IsLeft() && !Remote_IsRight())
        s_turn_kd = 0.6f;
    else
        s_turn_kd = 0.0f;

    /*--------- 2.5 超声波避障：障碍过近时覆盖速度目标 ---------*/
    /* 平衡车不能原地静止，采用后退远离策略；蓝牙遥控速度指令被覆盖 */
    if (s_obstacle_cm >= 0.0f)
    {
        if (s_obstacle_cm < OBSTACLE_STOP_CM)
            s_target_speed = SPEED_Y;           /* 很近：全力后退（正目标=后退）*/
        else if (s_obstacle_cm < OBSTACLE_SLOW_CM)
            s_target_speed = SPEED_Y / 2;       /* 较近：半速后退 */
    }

    /*--------- 3. 级联 PID 计算 ---------*/

    /* 速度环 PI：输出目标角度偏移 */
    s_velocity_out = PID_Velocity(s_velocity_kp, s_velocity_ki, s_target_speed, enc_l, enc_r);

    /*────────────────────────────────────────────────────────────────*/
    /* 速度环输出限幅                                                 */
    /*────────────────────────────────────────────────────────────────*/
    /* 物理约束：小车最大可控倾角约 ±15°                             */
    /* 安全范围：限制目标倾角偏移在 ±15° 以内（留有安全裕度）        */
    /*                                                              */
    /* 注意：速度环输出单位取决于 kp/ki 的定义                       */
    /*       如果速度环输出是"倾角偏移"，单位是"度"                */
    /*       需要通过 VOFA+ 观察实际输出范围                        */
    /*                                                              */
    /* 作用：                                                        */
    /* 1. 防止 PID 输出过大，导致目标倾角超出物理可控范围            */
    /* 2. 配合积分抗饱和，在积分累积阶段也强制限幅                   */
    /* 3. 确保小车在异常情况下（如卡住）不会输出极端值               */
    /*────────────────────────────────────────────────────────────────*/
    

    /*if (s_velocity_out > VELOCITY_OUT_MAX)
        s_velocity_out = VELOCITY_OUT_MAX;
    if (s_velocity_out < -VELOCITY_OUT_MAX)
        s_velocity_out = -VELOCITY_OUT_MAX;*/

    /* 直立环 PD：叠加速度环输出作为目标角度 */
    s_vertical_out = PID_Vertical(s_vertical_kp, s_vertical_kd,
                                  s_velocity_out + s_med_angle, roll, (float)gyrox);

    /* 转向环 PD：输出转向差速 */
    s_turn_out = PID_Turn(s_turn_kp, s_turn_kd, (float)gyroz, s_target_turn);

    /*--------- 4. 电机输出 ---------*/
    /* 左右电机差速驱动：MOTO1 = 垂直输出 - 转向输出 */
    s_moto1 = s_vertical_out - s_turn_out;
    s_moto2 = s_vertical_out + s_turn_out;

    /* PWM 限幅并输出 */
    Limit(&s_moto1, &s_moto2);
    Load(s_moto1, s_moto2);

    /* 倾倒保护：偏差超过 60 度时停机 */
    s_stopped = Stop(&s_med_angle, &roll);

    /* 倾倒后重置速度环积分，避免累积误差 */
    if (s_stopped) {
        PID_Velocity_Reset();
    }
}

/*============================================================================*/
/*                              访问接口                                       */
/*============================================================================*/

/**
 * @brief  获取左电机 PWM 输出值
 * @return PWM 占空比（-7200~7200）
 */
int Control_GetMoto1(void) { return s_moto1; }

/**
 * @brief  获取右电机 PWM 输出值
 * @return PWM 占空比（-7200~7200）
 */
int Control_GetMoto2(void) { return s_moto2; }

/**
 * @brief  获取直立环输出值（用于调试）
 * @return 直立环 PD 计算结果
 */
int Control_GetVerticalOut(void) { return s_vertical_out; }

/**
 * @brief  获取速度环输出值（用于调试）
 * @return 速度环 PI 计算结果（目标角度偏移）
 */
int Control_GetVelocityOut(void) { return s_velocity_out; }

/**
 * @brief  获取转向环输出值（用于调试）
 * @return 转向环 PD 计算结果
 */
int Control_GetTurnOut(void) { return s_turn_out; }

/**
 * @brief  获取目标速度（用于 VOFA+ 调试）
 * @return 目标速度（编码器脉冲/10ms）
 */
int Control_GetTargetSpeed(void)
{
    return s_target_speed;
}

/**
 * @brief  获取目标转向（用于 VOFA+ 调试）
 * @return 目标转向角速度
 */
int Control_GetTargetTurn(void)
{
    return s_target_turn;
}

/*============================================================================*/
/*                              PID 参数访问                                   */
/*============================================================================*/

float Control_GetVerticalKp(void) { return s_vertical_kp; }
float Control_GetVerticalKd(void) { return s_vertical_kd; }
float Control_GetVelocityKp(void) { return s_velocity_kp; }
float Control_GetVelocityKi(void) { return s_velocity_ki; }
float Control_GetTurnKp(void)     { return s_turn_kp; }
float Control_GetTurnKd(void)     { return s_turn_kd; }

void Control_SetVerticalKp(float kp) { s_vertical_kp = kp; }
void Control_SetVerticalKd(float kd) { s_vertical_kd = kd; }
void Control_SetVelocityKp(float kp) { s_velocity_kp = kp; }
void Control_SetVelocityKi(float ki) { s_velocity_ki = ki; }
void Control_SetTurnKp(float kp)     { s_turn_kp = kp; }
void Control_SetTurnKd(float kd)     { s_turn_kd = kd; }

/**
 * @brief  获取机械零点角度
 * @return 平衡时的自然倾斜角（度）
 */
float Control_GetMedAngle(void) { return s_med_angle; }

/**
 * @brief  设置机械零点角度（用于 VOFA+ 调试）
 * @param  angle: 新的机械零点角度（度）
 */
void Control_SetMedAngle(float angle) { s_med_angle = angle; }

/**
 * @brief  查询倾倒保护状态
 * @return 1=已倾倒，0=正常
 */
uint8_t Control_IsStopped(void) { return s_stopped; }

/**
 * @brief  超声波避障：设置障碍物距离（cm）
 * @param  cm 障碍距离；<0 表示无有效回波（无障碍），清除避障状态
 * @note   由 SR04_Task 周期调用（约 60ms），控制环每 10ms 读取
 */
void Control_SetObstacle(float cm)
{
    s_obstacle_cm = cm;
}
