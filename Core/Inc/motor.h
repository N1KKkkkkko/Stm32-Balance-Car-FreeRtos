#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"

void Load(int moto1, int moto2);
void Limit(int *motoA, int *motoB);
/* Stop: 倾倒保护，返回 1 表示已倾倒 */
uint8_t Stop(float *med_angle, float *angle);

#endif
