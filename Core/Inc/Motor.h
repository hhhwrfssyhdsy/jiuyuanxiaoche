//
// Created by LENOVO on 2024/9/15.
//

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint-gcc.h>

void Motor_Init(void);
void Motor1_SetSpeed(uint8_t Dir, uint16_t Speed);
void Motor2_SetSpeed(uint8_t Dir, uint16_t Speed);
void Car_Stop(void);
void Car_Forward(uint16_t Speed);
void Car_Backward(uint16_t Speed);
void Car_TurnLeft(uint16_t Speed);
void Car_TurnRight(uint16_t Speed);


#endif //XIAOCHE_MOTOR_H
