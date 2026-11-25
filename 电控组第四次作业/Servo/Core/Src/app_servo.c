#include "app_servo.h"
#include "main.h"
#include "OLED.h"
#include "Servo.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

#define KEY_DEBOUNCE_MS 20U
#define AUTO_INTERVAL_MS 1000U

typedef struct {
    uint16_t pin;
    GPIO_PinState raw;
    GPIO_PinState stable;
    uint32_t changed_at;
} Key;

static Key plus_key;
static Key minus_key;
static const uint16_t sequence[] = {0, 45, 90, 135, 180};
static uint8_t seq_index;
static uint16_t target;
static uint16_t pulse_us;
static uint32_t auto_at;
static uint8_t display_dirty;

static void Key_Init(Key *key, uint16_t pin, uint32_t now)
{
    key->pin = pin;
    /* A button held during boot is recognized after 20 ms. */
    key->raw = GPIO_PIN_SET;
    key->stable = GPIO_PIN_SET;
    key->changed_at = now;
}

/* One event on the debounced press edge. Holding does not repeat. */
static uint8_t Key_Update(Key *key, uint32_t now)
{
    GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOA, key->pin);
    if (sample != key->raw) {
        key->raw = sample;
        key->changed_at = now;
    }
    if (key->stable != key->raw &&
        (uint32_t)(now - key->changed_at) >= KEY_DEBOUNCE_MS) {
        key->stable = key->raw;
        return key->stable == GPIO_PIN_RESET;
    }
    return 0;
}

static void SetTarget(uint16_t angle)
{
    if (angle > 180U) return;
    target = angle;
    pulse_us = Servo_AngleToCCR((float)angle);
    Servo_SetAngle((float)angle);
    display_dirty = 1;
}

static void ApplyStep(int delta)
{
    /* Calculate in a SIGNED temporary. Reject BEFORE updating target.
     * Example: 180 + 15 is ignored; the next -15 still gives 165. */
    int candidate = (int)target + delta;
    if (candidate >= 0 && candidate <= 180) {
        SetTarget((uint16_t)candidate);
    }
}

static void ShowStatus(void)
{
    char uart_line[64];
    Servo_ShowOnOLED((float)target, pulse_us);

    int length = snprintf(uart_line, sizeof uart_line,
                          "Target=%03u CCR=%04u Pulse=%u.%03ums\r\n",
                          (unsigned)target, (unsigned)pulse_us,
                          (unsigned)(pulse_us / 1000U),
                          (unsigned)(pulse_us % 1000U));
    if (length > 0 && length < (int)sizeof uart_line) {
        /* Short blocking transmission only when the target changes.
         * No printf redirection / duplicate _write definition required. */
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_line,
                                (uint16_t)length, 20U);
    }
    display_dirty = 0;
}

void AppServo_Init(void)
{
    OLED_Init();
    seq_index = 0;
    SetTarget(sequence[seq_index]);
    /* Load CCR preload before starting so the first pulse is valid. */
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    ShowStatus();
    auto_at = HAL_GetTick();
    Key_Init(&plus_key, GPIO_PIN_1, auto_at);
    Key_Init(&minus_key, GPIO_PIN_2, auto_at);
}

void AppServo_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t plus_event = Key_Update(&plus_key, now);
    uint8_t minus_event = Key_Update(&minus_key, now);

    /* Simultaneous keys: no angle change, but pause the sequence.
     * An event is consumed even when ignored, so release never retriggers it. */
    if (plus_event && minus_key.raw == GPIO_PIN_SET &&
                      minus_key.stable == GPIO_PIN_SET) {
        ApplyStep(15);
    } else if (minus_event && plus_key.raw == GPIO_PIN_SET &&
                             plus_key.stable == GPIO_PIN_SET) {
        ApplyStep(-15);
    }

    if (plus_key.raw == GPIO_PIN_RESET || plus_key.stable == GPIO_PIN_RESET ||
        minus_key.raw == GPIO_PIN_RESET || minus_key.stable == GPIO_PIN_RESET) {
        auto_at = now;
    } else if ((uint32_t)(now - auto_at) >= AUTO_INTERVAL_MS) {
        seq_index = (uint8_t)((seq_index + 1U) % 5U);
        SetTarget(sequence[seq_index]);
        auto_at = now;
    }

    if (display_dirty) ShowStatus();
}
