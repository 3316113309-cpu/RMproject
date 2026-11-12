/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t run;
  uint8_t dir;
  uint16_t step_ms;
  uint8_t led_idx;
} LightState;

typedef struct
{
  uint8_t header;
  uint8_t run;
  uint8_t dir;
  uint8_t step_l;
  uint8_t step_h;
  uint8_t checksum;
} CommandPacket;

typedef struct
{
  uint8_t bytes[sizeof(CommandPacket)];
  uint8_t length;
} PacketParser;

typedef struct
{
  volatile uint8_t edge;
  uint8_t waiting_press;
  uint8_t waiting_release;
  uint32_t changed_at;
} KeyState;

typedef struct
{
  uint8_t run;
  uint8_t dir;
  uint16_t step_ms;
  uint8_t led_idx;
} ReportState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 512U
#define REPORT_QUEUE_SIZE 128U
#define KEY_DEBOUNCE_MS 20U
#define OLED_ALL_LINES 0x0FU
_Static_assert(sizeof(CommandPacket) == 6U, "Command packet must be 6 bytes");
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static LightState g_light = {1U, 0U, 300U, 0U};
static uint32_t g_last_step_tick;
static uint8_t g_oled_dirty = OLED_ALL_LINES;

static uint8_t g_rx_byte;
static volatile uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint8_t g_rx_overflow;
static PacketParser g_parser;

static KeyState g_key1;
static KeyState g_key2;

static uint8_t g_tx_buffer[64];
static volatile uint8_t g_tx_busy;
static ReportState g_reports[REPORT_QUEUE_SIZE];
static uint8_t g_report_head;
static uint8_t g_report_tail;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Light_WriteOutput(void);
static void Light_Advance(uint32_t now);
static void Light_ApplyPacket(const CommandPacket *packet, uint32_t now);
static void Key_Process(KeyState *key, uint16_t pin, uint32_t now);
static void Serial_Process(uint32_t now);
static void Serial_QueueReport(void);
static void Serial_TryReport(void);
static void OLED_RefreshOneLine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Light_WriteOutput(void)
{
  const uint16_t pins[3] = {LED1_Pin, LDE2_Pin, LED3_Pin};
  HAL_GPIO_WritePin(GPIOA, LED1_Pin | LDE2_Pin | LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, pins[g_light.led_idx], GPIO_PIN_RESET);
}

static void Light_Advance(uint32_t now)
{
  if (!g_light.run || (uint32_t)(now - g_last_step_tick) < g_light.step_ms)
  {
    return;
  }

  g_last_step_tick = now;
  g_light.led_idx = (uint8_t)((g_light.led_idx + (g_light.dir ? 2U : 1U)) % 3U);
  Light_WriteOutput();
  g_oled_dirty |= 0x08U;
}

static void Serial_QueueReport(void)
{
  uint8_t next = (uint8_t)((g_report_head + 1U) % REPORT_QUEUE_SIZE);
  if (next == g_report_tail)
  {
    return;
  }

  g_reports[g_report_head] = (ReportState){
    g_light.run, g_light.dir, g_light.step_ms, g_light.led_idx
  };
  g_report_head = next;
}

static void Light_ApplyPacket(const CommandPacket *packet, uint32_t now)
{
  uint8_t new_run = packet->run;
  uint8_t new_dir = packet->dir;
  uint16_t new_step = (uint16_t)packet->step_l | ((uint16_t)packet->step_h << 8);

  if (g_light.run != new_run)
  {
    g_oled_dirty |= 0x01U;
  }
  if (g_light.dir != new_dir)
  {
    g_oled_dirty |= 0x02U;
  }
  if (g_light.step_ms != new_step)
  {
    g_oled_dirty |= 0x04U;
  }

  g_light.run = new_run;
  g_light.dir = new_dir;
  g_light.step_ms = new_step;
  g_last_step_tick = now;
  Serial_QueueReport();
}

static void Key_Process(KeyState *key, uint16_t pin, uint32_t now)
{
  if (key->waiting_release)
  {
    if (HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_SET)
    {
      if (key->changed_at == 0U)
      {
        key->changed_at = now ? now : 1U;
      }
      else if ((uint32_t)(now - key->changed_at) >= KEY_DEBOUNCE_MS)
      {
        key->waiting_release = 0U;
        key->edge = 0U;
        key->changed_at = 0U;
      }
    }
    else
    {
      key->changed_at = 0U;
    }
    return;
  }

  if (!key->waiting_press && key->edge)
  {
    key->edge = 0U;
    key->waiting_press = 1U;
    key->changed_at = now;
  }

  if (key->waiting_press && (uint32_t)(now - key->changed_at) >= KEY_DEBOUNCE_MS)
  {
    key->waiting_press = 0U;
    if (HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_RESET)
    {
      key->waiting_release = 1U;
      key->changed_at = 0U;
      if (pin == KEY1_Pin)
      {
        g_light.dir ^= 1U;
        g_oled_dirty |= 0x02U;
      }
      else
      {
        g_light.run ^= 1U;
        g_last_step_tick = now;
        g_oled_dirty |= 0x01U;
      }
      Serial_QueueReport();
    }
  }
}

static void Serial_ParseByte(uint8_t byte, uint32_t now)
{
  if (g_parser.length == 0U)
  {
    if (byte != 0xAAU)
    {
      return;
    }
  }

  g_parser.bytes[g_parser.length++] = byte;
  if (g_parser.length == sizeof(g_parser.bytes))
  {
    const uint8_t *p = g_parser.bytes;
    CommandPacket packet = {p[0], p[1], p[2], p[3], p[4], p[5]};
    uint8_t checksum = (uint8_t)(packet.header + packet.run + packet.dir
                                 + packet.step_l + packet.step_h);
    uint16_t step = (uint16_t)packet.step_l | ((uint16_t)packet.step_h << 8);

    if (packet.header == 0xAAU && packet.checksum == checksum
        && packet.run <= 1U && packet.dir <= 1U
        && step >= 100U && step <= 2000U)
    {
      Light_ApplyPacket(&packet, now);
      g_parser.length = 0U;
    }
    else
    {
      uint8_t start = 1U;
      while (start < sizeof(g_parser.bytes) && p[start] != 0xAAU)
      {
        ++start;
      }
      g_parser.length = (uint8_t)(sizeof(g_parser.bytes) - start);
      for (uint8_t i = 0U; i < g_parser.length; ++i)
      {
        g_parser.bytes[i] = p[start + i];
      }
    }
  }
}

static void Serial_Process(uint32_t now)
{
  if (g_rx_overflow)
  {
    __disable_irq();
    g_rx_tail = g_rx_head;
    g_rx_overflow = 0U;
    __enable_irq();
    g_parser.length = 0U;
  }

  while (g_rx_tail != g_rx_head)
  {
    uint8_t byte = g_rx_buffer[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % RX_BUFFER_SIZE);
    Serial_ParseByte(byte, now);
  }
}

static void Serial_TryReport(void)
{
  if (g_tx_busy || g_report_tail == g_report_head)
  {
    return;
  }

  const ReportState *report = &g_reports[g_report_tail];
  int count = snprintf((char *)g_tx_buffer, sizeof(g_tx_buffer),
                       "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                       report->run ? "RUN" : "PAUSE",
                       report->dir ? "REV" : "FWD",
                       (unsigned)report->step_ms,
                       (unsigned)report->led_idx + 1U);
  if (count > 0 && count < (int)sizeof(g_tx_buffer))
  {
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart1, g_tx_buffer, (uint16_t)count) != HAL_OK)
    {
      g_tx_busy = 0U;
    }
    else
    {
      g_report_tail = (uint8_t)((g_report_tail + 1U) % REPORT_QUEUE_SIZE);
    }
  }
}

static void OLED_RefreshOneLine(void)
{
  if (g_oled_dirty == 0U)
  {
    return;
  }

  uint8_t line = 0U;
  while ((g_oled_dirty & (1U << line)) == 0U)
  {
    ++line;
  }
  g_oled_dirty &= (uint8_t)~(1U << line);

  char text[17];
  switch (line)
  {
    case 0U:
      (void)snprintf(text, sizeof(text), "STATE: %s", g_light.run ? "RUN" : "PAUSE");
      break;
    case 1U:
      (void)snprintf(text, sizeof(text), "DIR: %s", g_light.dir ? "REV" : "FWD");
      break;
    case 2U:
      (void)snprintf(text, sizeof(text), "STEP: %ums", (unsigned)g_light.step_ms);
      break;
    default:
      (void)snprintf(text, sizeof(text), "LED: %u", (unsigned)g_light.led_idx + 1U);
      break;
  }

  uint8_t used = (uint8_t)strlen(text);
  for (uint8_t i = used; i < 16U; ++i)
  {
    text[i] = ' ';
  }
  text[16] = '\0';
  OLED_ShowString((uint8_t)(line + 1U), 1U, text);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  Light_WriteOutput();
  g_last_step_tick = HAL_GetTick();
  if (HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    Serial_Process(now);
    Key_Process(&g_key1, KEY1_Pin, now);
    Key_Process(&g_key2, KEY2_Pin, now);
    Light_Advance(now);
    Serial_TryReport();
    OLED_RefreshOneLine();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY1_Pin)
  {
    g_key1.edge = 1U;
  }
  else if (GPIO_Pin == KEY2_Pin)
  {
    g_key2.edge = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next = (uint16_t)((g_rx_head + 1U) % RX_BUFFER_SIZE);
    if (next != g_rx_tail)
    {
      g_rx_buffer[g_rx_head] = g_rx_byte;
      g_rx_head = next;
    }
    else
    {
      g_rx_overflow = 1U;
    }
    (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    g_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    g_rx_overflow = 1U;
    (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
