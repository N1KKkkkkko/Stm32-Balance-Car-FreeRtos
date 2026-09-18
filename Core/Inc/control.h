#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

/* 控制环运行一次（10ms 周期） */
void Control_Run(void);

/* 获取控制输出 */
int Control_GetMoto1(void);
int Control_GetMoto2(void);
int Control_GetVerticalOut(void);
int Control_GetVelocityOut(void);
int Control_GetTurnOut(void);
int Control_GetTargetSpeed(void);
int Control_GetTargetTurn(void);

/* 获取/设置 PID 参数 */
float Control_GetVerticalKp(void);
float Control_GetVerticalKd(void);
float Control_GetVelocityKp(void);
float Control_GetVelocityKi(void);
float Control_GetTurnKp(void);
float Control_GetTurnKd(void);
void  Control_SetVerticalKp(float kp);
void  Control_SetVerticalKd(float kd);
void  Control_SetVelocityKp(float kp);
void  Control_SetVelocityKi(float ki);
void  Control_SetTurnKp(float kp);
void  Control_SetTurnKd(float kd);

/* 获取/设置机械零点 */
float Control_GetMedAngle(void);
void  Control_SetMedAngle(float angle);

/* 倾倒保护标志 */
uint8_t Control_IsStopped(void);

/* 超声波避障：设置障碍物距离（cm），<0 表示无有效回波（无障碍）*/
void Control_SetObstacle(float cm);

#endif
