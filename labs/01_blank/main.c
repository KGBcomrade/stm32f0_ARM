/*asm volatile("svc 0");
 * It is just simple and pure template project
 * It does absolutely nothing and indicates that toolchain
 * is installed correctly.
 */

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "ind7.h"
#include "stm32f0xx_ll_tim.h"


#define BUTT_TIME_START 0
#define BUTT_TIME_TRIG 300
#define BUTT_TIME_COMPLETE 301
#define LED_TIME_TRIG 1000


static int numi = 0;
static int8_t shc = 0;



static const int pow10[] = {1, 10, 100, 1000};




void gpio_config() {
    gpio_ind7_config();
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

}

static void rcc_config() {
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

static void systick_config() {
	LL_InitTick(48000000, 1000);

	LL_SYSTICK_EnableIT();

	NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler() {

	if((++shc) == 4)
		shc = 0;
	set_indicator(((numi / pow10[shc]) & 0xF) | ((shc == 3) << 4), shc);
		
	
	return;
}


static void timers_config(void)
{
    /*
     * Configure input channel
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

    /*
     * Setup timer to capture input mode
     */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 47999);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_FALLING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(TIM2);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM2);

    /*
     * Setup NVIC
     */
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    return;
}

void TIM2_IRQHandler(void)
{
    static tap = 0;
    static unsigned int t0 = 0;
    static unsigned int t = 0;
    static unsigned int dt = 0;
    if(tap == 0)
	    t0 = LL_TIM_IC_GetCaptureCH1(TIM2);
    else {
	    t = LL_TIM_IC_GetCaptureCH1(TIM2);
	    dt = t - t0;
	    if(dt < 0)
		    dt = -dt;
	    numi = dt;
	    t0 = 0;
	    t = 0;
	    LL_TIM_GenerateEvent_UPDATE(TIM2);
    }
    tap = (tap + 1) % 2;
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
    LL_TIM_ClearFlag_CC1(TIM2);
}

int main(void)
{
	rcc_config();
	gpio_config();
	
	timers_config();
	systick_config();	
	
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);

	while(1);

	return 0;
}
