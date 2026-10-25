# 电控组第二次培训作业

- **李粤**

- **_20263001254_**

- 首先，使用cubemx创建工程文件。在右侧图形化界面将PA0—PA2引脚设置为output后，systemcore中设置GPIO，采用低电平输出，userlabel设置LED0—LED2.

- 进入CLion编辑main.c文件。

- **==总体思路==**这段程序本质上是一个基于时间戳的状态机。它不会停下来等待500 ms，而是持续运行主循环，每次检查距离上次切换是否已经过去500 ms。

1.首先定义LED引脚数组：

static const uint16_t ledPins[] =
{
    LED0_Pin,
    LED1_Pin,
    LED2_Pin
};

2.ledPin数组保存三个LED的引脚编号：

ledPins[0] = LED0_Pin = PA0
ledPins[1] = LED1_Pin = PA1
ledPins[2] = LED2_Pin = PA2

后续通过下标即可选择LED，这样可以避免为每一个LED重复写一套程序。

3.定义 LED数量

#define LED_COUNT 3U

4.定义全部引脚

#define ALL_LED_PINS \
    (LED0_Pin | LED1_Pin | LED2_Pin)

5.定义流水灯间隔

#define LED_INTERVAL_MS 500U

6.设置状态变量

static uint8_t currentLed = 0U;
static uint32_t previousTick = 0U;（表示初始从PA0开始，`HAL_gettick返回STM32启动后经过的毫秒数返回类型为uint32_t）

7.我采用输出低电平点亮，初始为全部关闭

HAL_GPIO_WritePin(GPIOA, ALL_LED_PINS, GPIO_PIN_SET);

8.点亮第一个LED

HAL_GPIO_WritePin(GPIOA, ledPins[currentLed], GPIO_PIN_RESET);、

9.保存初始时间

previousTick = HAL_GetTick();

10.循环读取当前时间

currentTick = HAL_GetTick();

11.在循环中判断是否经过500ms

if ((uint32_t)(currentTick - previousTick) >= LED_INTERVAL_MS)

满足后：

更新previousTick

关闭当前LED

HAL_GPIO_WritePin(GPIOA, ledPins[currentLed], GPIO_PIN_SET);

12.切换至下一个led

currentLed++;

13.这当中还需要嵌套一个if判断作为循环至PA2时回到第一个LED

if (currentLed >= LED_COUNT)  

currentLed = 0U;  

- ==流程概括为：==

上电
 ↓
初始化PA0～PA2
 ↓
关闭全部LED
 ↓
点亮LED0并记录时间
 ↓
不停读取HAL_GetTick()
 ↓
是否经过500ms？
 ├─ 否：继续执行主循环其他任务
 └─ 是：关闭当前灯
         → 下标加1
         → 超过2则回到0
         → 点亮下一个灯


