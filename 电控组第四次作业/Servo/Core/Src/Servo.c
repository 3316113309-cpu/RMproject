/* Completed from STM32_Training_4th/03_Code/Servo.c; original unchanged. */
#include "Servo.h"
#include "OLED.h"
extern TIM_HandleTypeDef htim2;

uint16_t Servo_AngleToCCR(float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    /* TODO 1 completed: use the configured endpoints; round to nearest us. */
    return (uint16_t)(SERVO_MIN_US +
           angle / 180.0f * (SERVO_MAX_US - SERVO_MIN_US) + 0.5f);
}

void Servo_SetAngle(float angle)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_AngleToCCR(angle));
}

void Servo_ShowOnOLED(float angle, uint16_t ccr)
{
    static uint8_t labels_drawn;
    if (!labels_drawn) {
        OLED_ShowString(1, 1, "Target:000 deg");
        OLED_ShowString(2, 1, "CCR   :0000");
        OLED_ShowString(3, 1, "Pulse :0.000ms");
        labels_drawn = 1;
    }
    /* TODO 2 completed: fixed-width numbers avoid stale digits. */
    OLED_ShowNum(1, 8, (uint32_t)angle, 3);
    OLED_ShowNum(2, 8, ccr, 4);
    OLED_ShowNum(3, 8, ccr / 1000U, 1);
    OLED_ShowNum(3, 10, ccr % 1000U, 3);
}
