#ifndef __AI_TUNER_H
#define __AI_TUNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define AI_TUNER_START_BYTE     0xAA
#define AI_TUNER_END_BYTE       0x55
#define AI_TUNER_BUFFER_SIZE    256
#define AI_TUNER_MAX_PARAMS     16
#define AI_TUNER_UART_TIMEOUT   100

typedef enum {
    AI_STATUS_OK = 0,
    AI_STATUS_ERROR,
    AI_STATUS_INVALID_PARAM,
    AI_STATUS_BUSY,
    AI_STATUS_TIMEOUT
} AI_Status_t;

typedef enum {
    AI_CMD_NONE = 0,
    AI_CMD_SET_PARAM,
    AI_CMD_GET_PARAM,
    AI_CMD_START_TEST,
    AI_CMD_STOP_TEST,
    AI_CMD_GET_PERFORMANCE,
    AI_CMD_RESET_PARAMS,
    AI_CMD_SAVE_PARAMS,
    AI_CMD_GET_ALL_PARAMS,
    AI_CMD_SET_MODE,
    AI_CMD_STREAM_DATA,
    AI_CMD_SET_STREAM_INTERVAL  // 设置流数据发送间隔
} AI_Command_t;

typedef enum {
    AI_MODE_NORMAL = 0,
    AI_MODE_TUNING,
    AI_MODE_TESTING
} AI_TunerMode_t;

typedef struct {
    float balance_score;
    float stability_score;
    float response_score;
    float overshoot_score;
    float total_score;
    uint32_t test_duration_ms;
    uint16_t sample_count;
    float avg_angle_error;
    float max_angle_error;
    float avg_motor_output;
    float angle_variance;
    float gyro_variance;
    float velocity_out_variance;
    float pwm_high_freq_energy;
    float cost_function;
} AI_Performance_t;

typedef struct {
    char name[16];
    float *ptr;
    float min;
    float max;
    float default_val;
} AI_Param_t;

typedef struct {
    uint8_t is_running;
    uint32_t start_time;
    uint32_t duration_ms;
    uint32_t sample_count;
    
    float angle_sum;
    float angle_sq_sum;
    float angle_max;
    float angle_change_sum;
    
    float gyro_sum;
    float gyro_sq_sum;
    
    float velocity_out_sum;
    float velocity_out_sq_sum;
    
    float motor_sum;
    float pwm_diff_sum;
    
    float prev_angle;
    float prev_gyro;
    float prev_velocity_out;
    int prev_moto1;
} AI_TestState_t;

void AI_Tuner_Init(void);
void AI_Tuner_Process(void);
void AI_Tuner_UART_Handler(uint8_t data);
void AI_Tuner_Control_Hook(void);
void AI_Tuner_ProcessResponses(void);  // 处理响应队列
void AI_Tuner_ProcessMain(void);       // 主循环处理函数

AI_Status_t AI_Tuner_SetParam(uint8_t index, float value);
float AI_Tuner_GetParam(uint8_t index);
void AI_Tuner_GetAllParams(float *values, uint8_t *count);
void AI_Tuner_ResetParams(void);

void AI_Tuner_StartTest(uint32_t duration_ms);
void AI_Tuner_StopTest(void);
AI_Performance_t AI_Tuner_GetPerformance(void);

void AI_Tuner_SetMode(AI_TunerMode_t mode);
AI_TunerMode_t AI_Tuner_GetMode(void);

void AI_Tuner_SendResponse(AI_Command_t cmd, AI_Status_t status, const uint8_t *data, uint16_t len);
void AI_Tuner_SendPerformance(AI_Performance_t *perf);

void AI_Tuner_DmaSend(const uint8_t *data, uint16_t len);
void AI_Tuner_StartRxDma(void);

void AI_Tuner_SetErrLowOut(int err_raw, int err_filtered);
float AI_Tuner_GetFilterA(void);

// 判断是否是数据包起始字节
uint8_t AI_Tuner_IsPacketStart(uint8_t data);

// 判断是否正在接收数据包
uint8_t AI_Tuner_IsReceiving(void);

// 重置接收状态（用于蓝牙遥控命令优先处理）
void AI_Tuner_ResetRxState(void);

#ifdef __cplusplus
}
#endif

#endif
