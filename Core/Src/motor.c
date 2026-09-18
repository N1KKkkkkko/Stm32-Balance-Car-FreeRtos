#include "motor.h"

#define PWM_MAX 7200
#define PWM_MIN -7200
extern TIM_HandleTypeDef htim1;

static int abs_val(int p)
{
    return p > 0 ? p : -p;
}

void Load(int moto1, int moto2)
{
    if (moto1 < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, abs_val(moto1));

    if (moto2 < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs_val(moto2));
}

void Limit(int *motoA, int *motoB) //速度限制
{
    if (*motoA > PWM_MAX) *motoA = PWM_MAX;
    if (*motoA < PWM_MIN) *motoA = PWM_MIN;
    if (*motoB > PWM_MAX) *motoB = PWM_MAX;
    if (*motoB < PWM_MIN) *motoB = PWM_MIN;
}

/* 倾倒保护：偏差超过60度时停机，返回1表示已倾倒 */
uint8_t Stop(float *med_angle, float *angle)
{
    if (abs_val((int)(*angle - *med_angle)) > 60)
    {
        Load(0, 0);
        return 1;
    }
    return 0;
}
