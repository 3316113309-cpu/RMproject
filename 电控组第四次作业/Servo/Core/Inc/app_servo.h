#ifndef APP_SERVO_H
#define APP_SERVO_H

/* Call after MX_GPIO_Init, MX_USART1_UART_Init and MX_TIM2_Init. */
void AppServo_Init(void);
/* Call repeatedly from while (1); no one-second blocking delay. */
void AppServo_Task(void);

#endif
