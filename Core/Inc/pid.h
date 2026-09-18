#ifndef __PID_H__
#define __PID_H__

#define VELOCITY_OUT_MAX 30   

/* 纯数学 PID 函数，不依赖任何全局变量 */

/* 直立环 PD */
int PID_Vertical(float kp, float kd, float med_angle, float angle, float gyro_x);

/* 速度环 PI（内部保持积分状态） */
void PID_Velocity_Reset(void);
int  PID_Velocity(float kp, float ki, int target, int enc_l, int enc_r);

/* 转向环 PD */
int PID_Turn(float kp, float kd, float gyro_z, int target_turn);

#endif
