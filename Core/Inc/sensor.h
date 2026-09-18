#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

/* 传感器初始化（MPU6050 + 编码器） */
void Sensor_Init(void);

/* 传感器数据更新（从硬件读取，每10ms调用一次） */
void Sensor_Update(void);

/* 获取传感器数据（仅暴露实际使用的数据） */
float Sensor_GetRoll(void);
float Sensor_GetRollDiff(void);
short Sensor_GetGyroX(void);
short Sensor_GetGyroY(void);
short Sensor_GetGyroZ(void);
int   Sensor_GetEncoderLeft(void);
int   Sensor_GetEncoderRight(void);
float Sensor_GetDistance(void);

#endif
