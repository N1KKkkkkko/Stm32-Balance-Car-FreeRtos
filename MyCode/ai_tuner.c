/* AI Tuner module: disabled (replaced by Sensor/Control/Remote modules)
 * Kept for reference when AI auto-tuning is re-enabled in the future.
 */
#if 0
#include "ai_tuner.h"
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2, htim4;

extern float roll, pitch, yaw;
extern short gyrox, gyroy, gyroz;
extern int Encoder_Left, Encoder_Right;
extern int MOTO1, MOTO2;
extern float Med_Angle;
extern int Velocity_out;

extern float Vertical_Kp, Vertical_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;

static float filter_a = 0.7f;
static int Err_LowOut_global = 0;
static int Err_raw_global = 0;  // 原始速度误差（滤波前）

static AI_Param_t params[AI_TUNER_MAX_PARAMS];
static uint8_t param_count = 0;

static AI_TunerMode_t current_mode = AI_MODE_NORMAL;
static AI_TestState_t test_state;
static AI_Performance_t last_performance;

static uint8_t rx_buffer[AI_TUNER_BUFFER_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_state = 0;
static uint16_t rx_expected_len = 0;

static uint8_t tx_buffer[AI_TUNER_BUFFER_SIZE];

/* DMA TX double buffer */
#define AI_TUNER_TX_BUF_COUNT 2
static uint8_t tx_dma_buf[AI_TUNER_TX_BUF_COUNT][AI_TUNER_BUFFER_SIZE];
static volatile uint8_t tx_buf_busy[AI_TUNER_TX_BUF_COUNT] = {0};

/* DMA RX buffer (idle-line reception) */
#define AI_TUNER_RX_DMA_SIZE 256
static uint8_t rx_dma_buf[AI_TUNER_RX_DMA_SIZE];

static volatile uint8_t stream_enabled = 0;
static volatile uint8_t stream_trigger = 0;  // 流数据发送触发标志
static volatile uint8_t is_sending_response = 0;  // 标记是否正在发送响应
static volatile uint8_t stream_paused = 0;  // 流数据暂停标志
static uint32_t last_stream_time = 0;
static uint32_t stream_resume_time = 0;  // 流数据恢复时间
static uint32_t stream_interval = 16;   // 流数据发送间隔（默认16ms = 60Hz，可调整）

// 响应队列（避免在中断中阻塞发送）
#define RESPONSE_QUEUE_SIZE 4
typedef struct {
    uint8_t data[AI_TUNER_BUFFER_SIZE];
    uint16_t len;
    uint8_t valid;
} ResponsePacket_t;
static ResponsePacket_t response_queue[RESPONSE_QUEUE_SIZE];
static volatile uint8_t response_pending = 0;  // 标记有待发送的响应

static void AI_Tuner_RegisterParams(void);
static void AI_Tuner_ParsePacket(uint8_t cmd, uint8_t *data, uint16_t len);
static uint16_t AI_Tuner_BuildPacket(uint8_t cmd, const uint8_t *data, uint16_t data_len, uint8_t *out);
static void AI_Tuner_SendStreamData(void);

void AI_Tuner_Init(void)
{
    param_count = 0;
    rx_index = 0;
    rx_state = 0;
    current_mode = AI_MODE_NORMAL;
    stream_enabled = 0;
    stream_trigger = 0;
    is_sending_response = 0;
    response_pending = 0;
    memset(&test_state, 0, sizeof(test_state));
    memset(&last_performance, 0, sizeof(last_performance));
    memset(response_queue, 0, sizeof(response_queue));
    
    AI_Tuner_RegisterParams();
}

static void AI_Tuner_RegisterParams(void)
{
    AI_Param_t p;
    
    memset(&p, 0, sizeof(p));
    
    strcpy(p.name, "V_Kp");
    p.ptr = &Vertical_Kp;
    p.min = -500.0f;
    p.max = 500.0f;
    p.default_val = -120.0f;
    params[param_count++] = p;
    
    strcpy(p.name, "V_Kd");
    p.ptr = &Vertical_Kd;
    p.min = -10.0f;
    p.max = 10.0f;
    p.default_val = -0.75f;
    params[param_count++] = p;
    
    strcpy(p.name, "S_Kp");
    p.ptr = &Velocity_Kp;
    p.min = -5.0f;
    p.max = 5.0f;
    p.default_val = -1.1f;
    params[param_count++] = p;
    
    strcpy(p.name, "S_Ki");
    p.ptr = &Velocity_Ki;
    p.min = -0.1f;
    p.max = 0.1f;
    p.default_val = -0.0055f;
    params[param_count++] = p;
    
    strcpy(p.name, "T_Kp");
    p.ptr = &Turn_Kp;
    p.min = 0.0f;
    p.max = 50.0f;
    p.default_val = 10.0f;
    params[param_count++] = p;
    
    strcpy(p.name, "T_Kd");
    p.ptr = &Turn_Kd;
    p.min = 0.0f;
    p.max = 5.0f;
    p.default_val = 0.6f;
    params[param_count++] = p;
    
    strcpy(p.name, "Med_Angle");
    p.ptr = &Med_Angle;
    p.min = -30.0f;
    p.max = 30.0f;
    p.default_val = -2.0f;
    params[param_count++] = p;
    
    strcpy(p.name, "Filter_a");
    p.ptr = &filter_a;
    p.min = 0.1f;
    p.max = 0.9f;
    p.default_val = 0.7f;
    params[param_count++] = p;
}

void AI_Tuner_UART_Handler(uint8_t data)
{
    static uint16_t payload_len = 0;
    
    switch(rx_state) {
        case 0:
            if(data == AI_TUNER_START_BYTE) {
                rx_buffer[0] = data;
                rx_index = 1;
                rx_state = 1;
            }
            break;
            
        case 1:
            rx_buffer[rx_index++] = data;
            if(rx_index >= 4) {
                payload_len = rx_buffer[2] | (rx_buffer[3] << 8);
                // 帧头(1)+命令(1)+长度(2)+状态(1)+数据(payload_len)+帧尾(2)
                rx_expected_len = 5 + payload_len + 2;
                // FIX: 增加边界检查，防止缓冲区溢出
                if(rx_expected_len > AI_TUNER_BUFFER_SIZE) {
                    rx_state = 0;
                    rx_index = 0;
                    break;
                }
                rx_state = 2;
            }
            break;
            
        case 2:
            rx_buffer[rx_index++] = data;
            if(rx_index >= rx_expected_len) {
                if(rx_buffer[rx_index-1] == AI_TUNER_END_BYTE && 
                   rx_buffer[rx_index-2] == AI_TUNER_END_BYTE) {
                    uint8_t cmd = rx_buffer[1];
                    uint8_t *payload = &rx_buffer[5];  // 跳过帧头+命令+长度+状态
                    AI_Tuner_ParsePacket(cmd, payload, payload_len);
                }
                rx_state = 0;
                rx_index = 0;
            }
            break;
    }
}

static void AI_Tuner_ParsePacket(uint8_t cmd, uint8_t *data, uint16_t len)
{
    uint8_t response_data[128];
    uint16_t response_len = 0;
    AI_Status_t status = AI_STATUS_OK;
    
    switch(cmd) {
        case AI_CMD_SET_PARAM: {
            if(len >= 5) {
                uint8_t index = data[0];
                float value;
                memcpy(&value, &data[1], 4);
                status = AI_Tuner_SetParam(index, value);
            } else {
                status = AI_STATUS_INVALID_PARAM;
            }
            response_len = 0;
            break;
        }
        
        case AI_CMD_GET_PARAM: {
            if(len >= 1) {
                uint8_t index = data[0];
                float value = AI_Tuner_GetParam(index);
                memcpy(response_data, &value, 4);
                response_len = 4;
            } else {
                status = AI_STATUS_INVALID_PARAM;
            }
            break;
        }
        
        case AI_CMD_GET_ALL_PARAMS: {
            float all_params[AI_TUNER_MAX_PARAMS];
            uint8_t count;
            AI_Tuner_GetAllParams(all_params, &count);
            response_data[0] = count;
            memcpy(&response_data[1], all_params, count * sizeof(float));
            response_len = 1 + count * sizeof(float);
            break;
        }
        
        case AI_CMD_START_TEST: {
            uint32_t duration = 5000;
            if(len >= 4) {
                memcpy(&duration, data, 4);
            }
            AI_Tuner_StartTest(duration);
            response_len = 0;
            break;
        }
        
        case AI_CMD_STOP_TEST: {
            AI_Tuner_StopTest();
            response_len = 0;
            break;
        }
        
        case AI_CMD_GET_PERFORMANCE: {
            AI_Performance_t perf = AI_Tuner_GetPerformance();
            memcpy(response_data, &perf, sizeof(AI_Performance_t));
            response_len = sizeof(AI_Performance_t);
            break;
        }
        
        case AI_CMD_RESET_PARAMS: {
            AI_Tuner_ResetParams();
            response_len = 0;
            break;
        }
        
        case AI_CMD_SET_MODE: {
            if(len >= 1) {
                AI_Tuner_SetMode((AI_TunerMode_t)data[0]);
            }
            response_len = 0;
            break;
        }
        
        case AI_CMD_STREAM_DATA: {
            if(len >= 1) {
                // 先设置流数据状态
                uint8_t new_stream_state = data[0];
                
                // 如果要停止流数据，立即停止
                if(new_stream_state == 0) {
                    stream_enabled = 0;
                }
                
                // 发送响应包（使用命令码0x0A，但数据长度为0）
                response_len = 0;
                AI_Tuner_SendResponse((AI_Command_t)cmd, status, response_data, response_len);
                
                // 如果要启动流数据，在响应包发送后再启动
                if(new_stream_state == 1) {
                    stream_enabled = 1;
                }
                
                return;  // 直接返回，避免重复发送响应
            }
            response_len = 0;
            break;
        }
        
        case AI_CMD_SET_STREAM_INTERVAL: {
            // 设置流数据发送间隔（单位：ms）
            if(len >= 2) {
                uint16_t interval = data[0] | (data[1] << 8);
                if(interval < 10) interval = 10;   // 最小10ms (100Hz)
                if(interval > 1000) interval = 1000;  // 最大1000ms (1Hz)
                stream_interval = interval;
            }
            response_len = 0;
            break;
        }
        
        default:
            status = AI_STATUS_ERROR;
            break;
    }
    
    AI_Tuner_SendResponse((AI_Command_t)cmd, status, response_data, response_len);
}

void AI_Tuner_SendResponse(AI_Command_t cmd, AI_Status_t status, const uint8_t *data, uint16_t len)
{
    uint16_t packet_len = AI_Tuner_BuildPacket(cmd, data, len, tx_buffer);
    
    // 进入临界区
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    // 暂停流数据发送（收到命令时暂停200ms）
    stream_paused = 1;
    stream_resume_time = HAL_GetTick() + 200;
    
    // 将响应包放入队列（避免在中断中阻塞发送）
    for(int i = 0; i < RESPONSE_QUEUE_SIZE; i++) {
        if(!response_queue[i].valid) {
            memcpy(response_queue[i].data, tx_buffer, packet_len);
            response_queue[i].len = packet_len;
            response_queue[i].valid = 1;
            response_pending = 1;
            break;
        }
    }
    
    // 退出临界区
    __set_PRIMASK(primask);
}

/* Internal helper: check how many TX DMA buffers are busy */
static uint8_t AI_Tuner_TxBufBusyCount(void)
{
    uint8_t count = 0;
    for(int i = 0; i < AI_TUNER_TX_BUF_COUNT; i++) {
        if(tx_buf_busy[i]) count++;
    }
    return count;
}

/* DMA send with double buffering. Falls back to blocking if both buffers busy. */
void AI_Tuner_DmaSend(const uint8_t *data, uint16_t len)
{
    if(len == 0 || len > AI_TUNER_BUFFER_SIZE) return;

    int idx = -1;
    for(int i = 0; i < AI_TUNER_TX_BUF_COUNT; i++) {
        if(!tx_buf_busy[i]) {
            idx = i;
            break;
        }
    }

    if(idx < 0) {
        /* Both buffers busy: fall back to blocking transmit */
        HAL_UART_Transmit(&huart3, data, len, 100);
        return;
    }

    memcpy(tx_dma_buf[idx], data, len);
    tx_buf_busy[idx] = 1;
    HAL_UART_Transmit_DMA(&huart3, tx_dma_buf[idx], len);
}

// 处理响应队列（在主循环中调用）
void AI_Tuner_ProcessResponses(void)
{
    // 临时缓冲区
    uint8_t temp_buffer[RESPONSE_QUEUE_SIZE][AI_TUNER_BUFFER_SIZE];
    uint16_t temp_len[RESPONSE_QUEUE_SIZE] = {0};
    uint8_t temp_count = 0;

    // 进入临界区，复制待发送的响应包
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if(!response_pending) {
        __set_PRIMASK(primask);
        return;
    }

    // 设置发送标记，阻止流数据发送
    is_sending_response = 1;

    // 复制所有待发送的响应包
    for(int i = 0; i < RESPONSE_QUEUE_SIZE; i++) {
        if(response_queue[i].valid) {
            memcpy(temp_buffer[temp_count], response_queue[i].data, response_queue[i].len);
            temp_len[temp_count] = response_queue[i].len;
            temp_count++;
            response_queue[i].valid = 0;
        }
    }

    response_pending = 0;

    // 退出临界区
    __set_PRIMASK(primask);

    // 发送所有响应包（使用DMA双缓冲）
    for(int i = 0; i < temp_count; i++) {
        AI_Tuner_DmaSend(temp_buffer[i], temp_len[i]);
    }

    // 如果没有包被发送，立即清除标记
    if(temp_count == 0) {
        is_sending_response = 0;
    }
}

static uint16_t AI_Tuner_BuildPacket(uint8_t cmd, const uint8_t *data, uint16_t data_len, uint8_t *out)
{
    uint16_t idx = 0;
    
    out[idx++] = AI_TUNER_START_BYTE;
    out[idx++] = cmd;
    out[idx++] = data_len & 0xFF;        // 长度字段：纯数据长度
    out[idx++] = (data_len >> 8) & 0xFF;
    out[idx++] = (uint8_t)AI_STATUS_OK;  // 状态字节
    
    if(data && data_len > 0) {
        memcpy(&out[idx], data, data_len);
        idx += data_len;
    }
    
    out[idx++] = AI_TUNER_END_BYTE;
    out[idx++] = AI_TUNER_END_BYTE;
    
    return idx;
}

static void AI_Tuner_SendStreamData(void)
{
    // 如果正在发送响应，跳过本次流数据发送
    if(is_sending_response) {
        return;
    }

    // 如果所有TX DMA缓冲区都忙，跳过本次流数据发送
    if(AI_Tuner_TxBufBusyCount() >= AI_TUNER_TX_BUF_COUNT) {
        return;
    }

    uint16_t idx = 0;
    float velocity_out_f = (float)Velocity_out;  // 转换为float

    tx_buffer[idx++] = AI_TUNER_START_BYTE;
    tx_buffer[idx++] = AI_CMD_STREAM_DATA;
    tx_buffer[idx++] = 26;  // Payload length: 4+2+4+2+2+2+2+4+4 = 26
    tx_buffer[idx++] = 0;
    tx_buffer[idx++] = 0;

    memcpy(&tx_buffer[idx], &roll, 4); idx += 4;
    memcpy(&tx_buffer[idx], &gyrox, 2); idx += 2;
    memcpy(&tx_buffer[idx], &velocity_out_f, 4); idx += 4;  // 发送float类型
    memcpy(&tx_buffer[idx], &MOTO1, 2); idx += 2;
    memcpy(&tx_buffer[idx], &MOTO2, 2); idx += 2;
    memcpy(&tx_buffer[idx], &Encoder_Left, 2); idx += 2;
    memcpy(&tx_buffer[idx], &Encoder_Right, 2); idx += 2;
    memcpy(&tx_buffer[idx], &Err_raw_global, 4); idx += 4;      // 原始误差
    memcpy(&tx_buffer[idx], &Err_LowOut_global, 4); idx += 4;   // 滤波后误差

    tx_buffer[idx++] = AI_TUNER_END_BYTE;
    tx_buffer[idx++] = AI_TUNER_END_BYTE;

    AI_Tuner_DmaSend(tx_buffer, idx);
}

void AI_Tuner_SendPerformance(AI_Performance_t *perf)
{
    uint16_t packet_len = AI_Tuner_BuildPacket(AI_CMD_GET_PERFORMANCE,
                                               (uint8_t*)perf,
                                               sizeof(AI_Performance_t),
                                               tx_buffer);
    AI_Tuner_DmaSend(tx_buffer, packet_len);
}

AI_Status_t AI_Tuner_SetParam(uint8_t index, float value)
{
    if(index >= param_count) return AI_STATUS_INVALID_PARAM;
    
    if(value < params[index].min) value = params[index].min;
    if(value > params[index].max) value = params[index].max;
    
    *params[index].ptr = value;
    
    return AI_STATUS_OK;
}

float AI_Tuner_GetParam(uint8_t index)
{
    if(index >= param_count) return 0.0f;
    return *params[index].ptr;
}

void AI_Tuner_GetAllParams(float *values, uint8_t *count)
{
    *count = param_count;
    for(uint8_t i = 0; i < param_count; i++) {
        values[i] = *params[i].ptr;
    }
}

void AI_Tuner_ResetParams(void)
{
    for(uint8_t i = 0; i < param_count; i++) {
        *params[i].ptr = params[i].default_val;
    }
}

void AI_Tuner_SetMode(AI_TunerMode_t mode)
{
    current_mode = mode;
}

AI_TunerMode_t AI_Tuner_GetMode(void)
{
    return current_mode;
}

void AI_Tuner_StartTest(uint32_t duration_ms)
{
    memset(&test_state, 0, sizeof(test_state));
    test_state.is_running = 1;
    test_state.start_time = HAL_GetTick();
    test_state.duration_ms = duration_ms;
    test_state.prev_angle = roll;
    test_state.prev_gyro = gyrox;
    test_state.prev_velocity_out = Velocity_out;
    test_state.prev_moto1 = MOTO1;
    current_mode = AI_MODE_TESTING;
}

void AI_Tuner_StopTest(void)
{
    if(test_state.is_running) {
        test_state.is_running = 0;
        current_mode = AI_MODE_NORMAL;
        
        last_performance.test_duration_ms = HAL_GetTick() - test_state.start_time;
        last_performance.sample_count = test_state.sample_count;
        
        if(test_state.sample_count > 0) {
            last_performance.avg_angle_error = test_state.angle_sum / test_state.sample_count;
            last_performance.max_angle_error = test_state.angle_max;
            last_performance.avg_motor_output = test_state.motor_sum / test_state.sample_count;
            
            float mean_angle = test_state.angle_sum / test_state.sample_count;
            last_performance.angle_variance = test_state.angle_sq_sum / test_state.sample_count - mean_angle * mean_angle;
            if(last_performance.angle_variance < 0) last_performance.angle_variance = 0;
            last_performance.angle_variance = sqrtf(last_performance.angle_variance);
            
            float mean_gyro = test_state.gyro_sum / test_state.sample_count;
            last_performance.gyro_variance = test_state.gyro_sq_sum / test_state.sample_count - mean_gyro * mean_gyro;
            if(last_performance.gyro_variance < 0) last_performance.gyro_variance = 0;
            last_performance.gyro_variance = sqrtf(last_performance.gyro_variance);
            
            float mean_vel = test_state.velocity_out_sum / test_state.sample_count;
            last_performance.velocity_out_variance = test_state.velocity_out_sq_sum / test_state.sample_count - mean_vel * mean_vel;
            if(last_performance.velocity_out_variance < 0) last_performance.velocity_out_variance = 0;
            last_performance.velocity_out_variance = sqrtf(last_performance.velocity_out_variance);
            
            last_performance.pwm_high_freq_energy = test_state.pwm_diff_sum / (test_state.sample_count - 1);
        }
        
        last_performance.balance_score = 100.0f / (1.0f + last_performance.angle_variance * 10.0f);
        last_performance.stability_score = 100.0f / (1.0f + last_performance.gyro_variance * 0.01f);
        last_performance.response_score = 100.0f;
        last_performance.overshoot_score = 100.0f / (1.0f + last_performance.max_angle_error * 2.0f);
        
        last_performance.total_score = 
            last_performance.balance_score * 0.3f +
            last_performance.stability_score * 0.3f +
            last_performance.response_score * 0.2f +
            last_performance.overshoot_score * 0.2f;
        
        last_performance.cost_function = 
            1.0f * last_performance.angle_variance +
            0.5f * last_performance.gyro_variance * 0.01f +
            0.3f * last_performance.velocity_out_variance +
            0.2f * last_performance.pwm_high_freq_energy;
        
        AI_Tuner_SendPerformance(&last_performance);
    }
}

AI_Performance_t AI_Tuner_GetPerformance(void)
{
    return last_performance;
}

void AI_Tuner_Control_Hook(void)
{
    uint32_t now = HAL_GetTick();
    
    // 设置流数据发送触发标志（在主循环中处理）
    // 使用stream_interval控制发送频率
    if(stream_enabled && (now - last_stream_time >= stream_interval)) {
        last_stream_time = now;
        stream_trigger = 1;
    }
    
    // 测试状态数据采集（在中断中进行，因为需要实时性）
    if(!test_state.is_running) return;
    
    uint32_t elapsed = now - test_state.start_time;
    if(elapsed >= test_state.duration_ms) {
        AI_Tuner_StopTest();
        return;
    }
    
    float angle_error = roll - Med_Angle;
    float angle_error_abs = fabsf(angle_error);
    
    /* FIX: 倒地/失控检测 - 角度误差超过30度认为已倒地，提前结束测试并给0分 */
    if(angle_error_abs > 30.0f) {
        test_state.is_running = 0;
        current_mode = AI_MODE_NORMAL;
        
        memset(&last_performance, 0, sizeof(last_performance));
        last_performance.test_duration_ms = elapsed;
        last_performance.total_score = 0.0f;
        last_performance.balance_score = 0.0f;
        last_performance.stability_score = 0.0f;
        last_performance.response_score = 0.0f;
        last_performance.overshoot_score = 0.0f;
        last_performance.sample_count = test_state.sample_count;
        
        /* 不直接发送，等待上位机GET_PERFORMANCE请求时通过响应队列发送 */
        return;
    }
    
    test_state.angle_sum += angle_error_abs;
    test_state.angle_sq_sum += angle_error * angle_error;
    
    if(angle_error_abs > test_state.angle_max) {
        test_state.angle_max = angle_error_abs;
    }
    
    test_state.gyro_sum += gyrox;
    test_state.gyro_sq_sum += (float)gyrox * gyrox;
    
    test_state.velocity_out_sum += Velocity_out;
    test_state.velocity_out_sq_sum += (float)Velocity_out * Velocity_out;
    
    float motor_out = fabsf((float)(MOTO1 + MOTO2) / 2.0f);
    test_state.motor_sum += motor_out;
    
    if(test_state.sample_count > 0) {
        float pwm_diff = fabsf((float)(MOTO1 - test_state.prev_moto1));
        test_state.pwm_diff_sum += pwm_diff;
    }
    
    test_state.prev_angle = roll;
    test_state.prev_gyro = gyrox;
    test_state.prev_velocity_out = Velocity_out;
    test_state.prev_moto1 = MOTO1;
    
    test_state.sample_count++;
}

// 主循环处理函数（处理响应队列和流数据发送）
void AI_Tuner_ProcessMain(void)
{
    uint32_t now = HAL_GetTick();
    
    // 检查是否应该恢复流数据发送
    if(stream_paused && now >= stream_resume_time) {
        stream_paused = 0;
    }
    
    // 处理响应队列（优先发送响应包）
    AI_Tuner_ProcessResponses();
    
    // 发送流数据（如果未暂停且没有待发送的响应）
    if(stream_trigger && !stream_paused && !response_pending) {
        stream_trigger = 0;
        AI_Tuner_SendStreamData();
    }
}

void AI_Tuner_Process(void)
{
    static uint32_t last_send = 0;
    uint32_t now = HAL_GetTick();
    
    if(current_mode == AI_MODE_TUNING) {
        if(now - last_send > 100) {
            last_send = now;
            
            uint8_t data[16];
            int16_t angle_int = (int16_t)(roll * 100);
            int16_t gyro_int = gyrox;
            int16_t motor_l = MOTO1;
            int16_t motor_r = MOTO2;
            
            memcpy(&data[0], &angle_int, 2);
            memcpy(&data[2], &gyro_int, 2);
            memcpy(&data[4], &motor_l, 2);
            memcpy(&data[6], &motor_r, 2);
            
            AI_Tuner_SendResponse(AI_CMD_GET_PERFORMANCE, AI_STATUS_OK, data, 8);
        }
    }
}

void AI_Tuner_SetErrLowOut(int err_raw, int err_filtered)
{
    Err_raw_global = err_raw;
    Err_LowOut_global = err_filtered;
}

float AI_Tuner_GetFilterA(void)
{
    return filter_a;
}

// 判断是否是数据包起始字节
uint8_t AI_Tuner_IsPacketStart(uint8_t data)
{
    return (data == AI_TUNER_START_BYTE);
}

// 判断是否正在接收数据包
uint8_t AI_Tuner_IsReceiving(void)
{
    return (rx_state != 0);
}

// 重置接收状态（用于蓝牙遥控命令优先处理）
void AI_Tuner_ResetRxState(void)
{
    rx_state = 0;
    rx_index = 0;
    rx_expected_len = 0;
}

/* ===================== DMA UART Callbacks ===================== */

extern uint8_t Bluetooth_data;
extern uint8_t Fore, Back, Left, Right;

static void AI_Tuner_ProcessRxByte(uint8_t data)
{
    /* Bluetooth remote commands have highest priority */
    /* FIX: 只有在非接收状态下才处理蓝牙命令，避免AI数据包中的0x01/0x03/0x05/0x07被误判 */
    if(!AI_Tuner_IsReceiving() && (data == 0x00 || data == 0x01 || data == 0x05 || data == 0x03 || data == 0x07))
    {
        AI_Tuner_ResetRxState();
        Bluetooth_data = data;
        if(Bluetooth_data == 0x00)      Fore=0, Back=0, Left=0, Right=0;
        else if(Bluetooth_data == 0x01) Fore=1, Back=0, Left=0, Right=0;
        else if(Bluetooth_data == 0x05) Fore=0, Back=1, Left=0, Right=0;
        else if(Bluetooth_data == 0x03) Fore=0, Back=0, Left=0, Right=1;
        else if(Bluetooth_data == 0x07) Fore=0, Back=0, Left=1, Right=0;
        else                            Fore=0, Back=0, Left=0, Right=0;
    }
    else if(data == AI_TUNER_START_BYTE || AI_Tuner_IsReceiving())
    {
        AI_Tuner_UART_Handler(data);
    }
    /* else: ignore invalid data */
}

/* Start idle-line DMA reception */
void AI_Tuner_StartRxDma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buf, AI_TUNER_RX_DMA_SIZE);
}

/* UART RX Event callback: disabled, moved to stm32f1xx_it.c (Remote module)
 * Kept for reference when AI Tuner is re-enabled in the future.
 */
#if 0
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        /* Copy data to temp buffer first, then restart DMA immediately
           to minimize the window where incoming bytes could be lost. */
        uint8_t temp[AI_TUNER_RX_DMA_SIZE];
        uint16_t copy_len = Size;
        if(copy_len > AI_TUNER_RX_DMA_SIZE) copy_len = AI_TUNER_RX_DMA_SIZE;
        memcpy(temp, rx_dma_buf, copy_len);

        /* Restart reception for next frame (Normal mode) */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buf, AI_TUNER_RX_DMA_SIZE);

        /* Process copied data */
        for(uint16_t i = 0; i < copy_len; i++) {
            AI_Tuner_ProcessRxByte(temp[i]);
        }
    }
}
#endif

/* UART RX complete callback: disabled, moved to stm32f1xx_it.c (Remote module)
 * Kept for reference when AI Tuner is re-enabled in the future.
 */
#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        uint8_t temp[AI_TUNER_RX_DMA_SIZE];
        memcpy(temp, rx_dma_buf, AI_TUNER_RX_DMA_SIZE);

        /* Restart reception */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buf, AI_TUNER_RX_DMA_SIZE);

        for(uint16_t i = 0; i < AI_TUNER_RX_DMA_SIZE; i++) {
            AI_Tuner_ProcessRxByte(temp[i]);
        }
    }
}
#endif

/* UART TX complete callback: mark double buffer as free */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        for(int i = 0; i < AI_TUNER_TX_BUF_COUNT; i++) {
            if(huart->pTxBuffPtr == tx_dma_buf[i]) {
                tx_buf_busy[i] = 0;
                break;
            }
        }
        /* If no more TX buffers busy, clear response-sending flag */
        uint8_t any_busy = 0;
        for(int i = 0; i < AI_TUNER_TX_BUF_COUNT; i++) {
            if(tx_buf_busy[i]) { any_busy = 1; break; }
        }
        if(!any_busy) {
            is_sending_response = 0;
        }
    }
}

/* UART error callback: restart reception on errors */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        /* Restart idle-line reception */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buf, AI_TUNER_RX_DMA_SIZE);
    }
}
#endif /* AI Tuner disabled */
