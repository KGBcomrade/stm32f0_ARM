/*
 * This example demonstrates using timers with encoder
 */

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "ind7.h"

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

static int numi = 0;
static int sound = 1;

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

/*
 * Clock on GPIOC and set two led pins
 */
static void gpio_config(void)
{
    gpio_ind7_config();
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    return;
}

/*
 * Configure timer to encoder mode
 */
static void timers_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);


    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_FALLING);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2,
                          LL_TIM_IC_POLARITY_FALLING);
    //LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    //LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_SetAutoReload(TIM2, 0xFFFF);
    LL_TIM_EnableCounter(TIM2);

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);
    LL_TIM_SetPrescaler(TIM16, 47999);
    LL_TIM_IC_SetFilter(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_IC_SetPolarity(TIM16, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_FALLING);
    LL_TIM_IC_SetActiveInput(TIM16, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(TIM16);
    LL_TIM_EnableCounter(TIM16);

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    
    NVIC_EnableIRQ(TIM16_IRQn);
    NVIC_SetPriority(TIM16_IRQn, 1);
   
     
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetPrescaler(TIM3, 47999);
    LL_TIM_SetAutoReload(TIM3, 999);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
    LL_TIM_SetPrescaler(TIM14, 47999);
    LL_TIM_SetCounterMode(TIM14, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(TIM14);
    LL_TIM_EnableCounter(TIM14);

    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 2);

    //
    return;
}

void TIM14_IRQHandler() {
	if (sound) 
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
	LL_TIM_ClearFlag_UPDATE(TIM14);
}

void TIM3_IRQHandler() {
	sound ^= 1;
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
	LL_TIM_ClearFlag_UPDATE(TIM3);
}

void TIM16_IRQHandler() {
    static tap = 0;
    static unsigned int t0 = 0;
    static unsigned int t = 0;
    static unsigned int dt = 0;
    if(tap == 0)
	    t0 = LL_TIM_IC_GetCaptureCH1(TIM16);
    else {
	    t = LL_TIM_IC_GetCaptureCH1(TIM16);
	    dt = t - t0;
	    if(dt < 0)
		    dt = -dt;
	    //numi = dt;
	    t0 = 0;
	    t = 0;
	    
    	    LL_TIM_SetAutoReload(TIM3, dt);
	    LL_TIM_GenerateEvent_UPDATE(TIM3);

	    LL_TIM_GenerateEvent_UPDATE(TIM16);
    }
    tap = (tap + 1) % 2;
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
    LL_TIM_ClearFlag_CC1(TIM16);
}
	

static void systick_config() {
	LL_InitTick(48000000, 1000);

	LL_SYSTICK_EnableIT();

	NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler() {
	static int8_t shc = 0;
	static const int pow10[] = {1, 10, 100, 1000};
	static int freq = 1, pfreq = 0;
	numi = freq;
	if((++shc) == 4)
		shc = 0;
	set_indicator(((numi / pow10[shc]) & 0xF) | ((shc == 3) << 4), shc);
	freq = LL_TIM_GetCounter(TIM2);
	if(freq == 0) {
    		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM14);
	} else {
    		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

		LL_TIM_SetAutoReload(TIM14, freq);
	}
	if(freq != pfreq) {
		pfreq = freq;
		LL_TIM_GenerateEvent_UPDATE(TIM14);
	}
	
	return;
}

/*
 * Turn on Green led when turn encoder to the right
 * Turn on Blue led when turn encoder to the left
 */
int main(void)
{
    rcc_config();
    gpio_config();
    timers_config();
    systick_config();

    
    return 0;
}
