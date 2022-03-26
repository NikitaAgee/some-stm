//#define TURN_ON_CONTACT_DEBOUNCER

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"

#include "stdint.h"

static void set_4indicator(uint16_t num);

/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */
static void rcc_config()
{
    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    /* Enable HSI and wait for activation*/
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    /* Set APB1 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    /* Update CMSIS variable (which can be updated also
     * through SystemCoreClockUpdate function) */
    SystemCoreClock = 48000000;
}

//7segments port define
#define SEG_A  LL_GPIO_PIN_0
#define SEG_B  LL_GPIO_PIN_1
#define SEG_C  LL_GPIO_PIN_2
#define SEG_D  LL_GPIO_PIN_3
#define SEG_E  LL_GPIO_PIN_4
#define SEG_F  LL_GPIO_PIN_5
#define SEG_G  LL_GPIO_PIN_6
#define FREES  0

#define GND_D1 LL_GPIO_PIN_10
#define GND_D2 LL_GPIO_PIN_9
#define GND_D3 LL_GPIO_PIN_8
#define GND_D4 LL_GPIO_PIN_7

/*
 * Clock on GPIOC and set two led pins
 */
static void gpio_config(void)
{   
    //кнопка
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    /*
     * Init port for indicator
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, SEG_A, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_B, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_C, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_D, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_E, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_F, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, SEG_G, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, GND_D1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, GND_D2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, GND_D3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, GND_D4, LL_GPIO_MODE_OUTPUT);
    return;
}


uint8_t button_status = 0;

uint64_t button_delay_counter = 0;

uint16_t number = 1234;

#define BUTTON_DELAY 10

#define BUTTON_PUSH 1

#define BUTTON_PUSHED 1 << 1

static void exti_config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    return;
}


static void systick_config(void)
{
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}

void EXTI0_1_IRQHandler(void)
{
    if(!(button_status & BUTTON_PUSH))
    {
        button_status = BUTTON_PUSH;
    }
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    return;        
}

void SysTick_Handler(void)
{   
    if((button_status & BUTTON_PUSH) && (button_delay_counter < BUTTON_DELAY))
    {
        button_delay_counter++;
    }
    else if(button_status & BUTTON_PUSH)
    {
        if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0))
        {
            if(!(button_status & BUTTON_PUSHED))
            {
                number += 1;
                button_status += BUTTON_PUSHED;
            }
        }
        else
        {
            button_delay_counter = 0;
            button_status = 0;
        }
    }
    return;
}

static void set_indicator(uint8_t number, uint8_t poz)
{
    /*
     * Put all pins for indicator together (for segments only)
     */
    static uint32_t mask = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G | GND_D1 | GND_D2 | GND_D3 | GND_D4;
    /*
     * For simplicity there are only decoded values for the first 4 numbers
     */
    static const uint32_t decoder[] = {
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | FREES, // 0
        FREES | SEG_B | SEG_C | FREES | FREES | FREES | FREES, // 1
        SEG_A | SEG_B | FREES | SEG_D | SEG_E | FREES | SEG_G, // 2
        SEG_A | SEG_B | SEG_C | SEG_D | FREES | FREES | SEG_G, // 3
        FREES | SEG_B | SEG_C | FREES | FREES | SEG_F | SEG_G, // 4
        SEG_A | FREES | SEG_C | SEG_D | FREES | SEG_F | SEG_G, // 5
        SEG_A | FREES | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 6
        SEG_A | SEG_B | SEG_C | FREES | FREES | FREES | FREES, // 7
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 8
        SEG_A | SEG_B | SEG_C | SEG_D | FREES | SEG_F | SEG_G  // 9
    };

    static const uint32_t poz_decoder[] = {
        FREES  | GND_D2 | GND_D3 | GND_D4,
        GND_D1 | FREES  | GND_D3 | GND_D4,
        GND_D1 | GND_D2 | FREES  | GND_D4,
        GND_D1 | GND_D2 | GND_D3 | FREES
    };
    const uint8_t max_num = sizeof(decoder) / sizeof(uint32_t);
    uint32_t port_state = 0;



    /*
     * Read current state and do not change pins that are not related to
     * indicator (that is done by using masking)
     */
    port_state = LL_GPIO_ReadOutputPort(GPIOB);
    /*
     * Example:
     * 01100101 <= Input
     * mask = 111 (pins allowed to be changed)
     * ~mask = 11111000 (inverted mask sets remaing bits to one)
     * result = result & ~mask (zero only first three bits)
     * result = result | 001 (modify first three bits)
     * result -> 01100001
     */
    port_state = (port_state & ~mask) | decoder[number % max_num] | poz_decoder[poz];
    LL_GPIO_WriteOutputPort(GPIOB, port_state);
    return;
}

static void set_4indicator(uint16_t num)
{

    uint32_t port_state = 0;

    uint8_t i = 0;
    for(i = 0; i < 4; i++, num = num / 10)
    {
        set_indicator(num % 10, i);
    }

    return;
}

int main(void)
{
    int i = 0;

    rcc_config();
    gpio_config();
    exti_config();
    systick_config();

    while(1)
    {
        set_4indicator(number);
    }
    return 0;
}
