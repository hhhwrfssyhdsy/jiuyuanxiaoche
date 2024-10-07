
#include "tim.h"
#include "gpio.h"
#include "Motor.h"

void Motor_Init(void)
{
    //GPIO以及计时器初始化
    MX_GPIO_Init();
    MX_TIM2_Init();


    //开启PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
}

//电机控制，DIR=1表示前进，DIR=0表示后退
void Motor1_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000; // Adjust to fit the 16-bit register
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Speed);
    }
}

void Motor2_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000; // Adjust to fit the 16-bit register
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Speed);
    }
}

//小车的行进
void Car_Stop(void)
{
    Motor1_SetSpeed(1,0);
    Motor2_SetSpeed(1,0);
}

void Car_Forward(uint16_t Speed)
{
    Motor1_SetSpeed(1,Speed);
    Motor2_SetSpeed(1,Speed);
}

void Car_Backward(uint16_t Speed)
{
    Motor1_SetSpeed(0,Speed);
    Motor2_SetSpeed(0,Speed);
}

void Car_TurnLeft(uint16_t Speed)
{
    Motor1_SetSpeed(0,Speed);
    Motor2_SetSpeed(1,Speed);
}

void Car_TurnRight(uint16_t Speed)
{
    Motor1_SetSpeed(1,Speed);
    Motor2_SetSpeed(0,Speed);
}

