#ifndef SERVO_H
#define SERVO_H
#include "stm32f1xx_hal.h"

/* Completed from the supplied STM32_Training_4th/03_Code/Servo.h. */
#define SERVO_MIN_US 550U
#define SERVO_MAX_US 2450U
uint16_t Servo_AngleToCCR(float angle);
void Servo_SetAngle(float angle);
void Servo_ShowOnOLED(float angle, uint16_t ccr);
#endif
