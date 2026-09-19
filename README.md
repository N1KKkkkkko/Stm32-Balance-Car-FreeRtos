# 基于 STM32F103 的 FreeRTOS 两轮平衡车

## 项目简介

本项目是一个基于 STM32F103 单片机和 FreeRTOS 实时操作系统的两轮自平衡小车。通过 MPU6050 获取姿态数据，使用 PID 算法控制电机，蓝牙进行操控，实现小车直立平衡、行进与转向。

## 演示
<img width="360" height="197" alt="平衡车演示" src="https://github.com/user-attachments/assets/1d4c31c9-5916-4a3a-9e41-5a6f76aea036" />


## 硬件平台

- MCU：STM32F103C8T6
- 姿态传感器：MPU6050（DMP）
- 电机驱动：TB6612 
- 显示：OLED（I2C）
- 遥控：蓝牙 / 无线串口
- 其他：编码器电机、超声波等

## 软件架构

- 基于 STM32 HAL 库开发
- FreeRTOS 任务划分：
  - 控制任务
  - 通讯任务
  - oled显示任务
   >优先级：控制任务>通讯任务>oled显示任务
- 使用 CMSIS-RTOS v2 封装

## 功能

- 直立平衡
- 前进/后退/转向
- 速度闭环控制
- OLED 实时显示姿态与参数
- 支持蓝牙遥控与参数调试
- 超声波防撞

## 编译与烧录

1. 使用 Keil MDK 打开 `MDK-ARM/OLED.uvprojx`
2. 编译工程
3. 通过 ST-Link 或串口烧录

---

## 演示视频链接：https://b23.tv/JmnlMby

## 实物图

<img width="1280" height="705" alt="bee0d3432194a92507a24ee71392079f_720" src="https://github.com/user-attachments/assets/f399bc1b-3e38-4d1f-9eb3-dc2f4be50147" />

---

联系方式：1995466@qq.com



