#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdint.h>

/* 蓝牙模块初始化：创建流缓冲 + 启动 USART3 DMA 空闲接收（在 CommTask 中调用） */
void Remote_Init(void);

/* 蓝牙数据处理任务体：阻塞读取流缓冲并解析指令（在 CommTask 循环中调用） */
void Remote_TaskHandler(void);

/* 解析单字节遥控指令（0x00/0x01/0x03/0x05/0x07） */
void Remote_ParseByte(uint8_t data);

/* 查询遥控状态 */
uint8_t Remote_IsForward(void);
uint8_t Remote_IsBackward(void);
uint8_t Remote_IsLeft(void);
uint8_t Remote_IsRight(void);

#endif
