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


#define BUTT_TIME_START 0
#define BUTT_TIME_TRIG 300
#define BUTT_TIME_COMPLETE 301
#define LED_TIME_TRIG 1000


static int numi = 2;
static int secc = 0;
static int8_t shc = 0;
static int decc = 201;
static int incc = 201;


static const int pow10[] = {1, 10, 100, 1000};




void gpio_config() {
	gpio_ind7_config();
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);	

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);



	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_OPENDRAIN);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);

}

static void rcc_config() {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    SystemCoreClock = 48000000;
}



static void systick_config() {
	LL_InitTick(48000000, 1000);

	LL_SYSTICK_EnableIT();

	NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler() {
	secc++;
	if(secc == LED_TIME_TRIG) {
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
		secc = 0;
	}
	if((shc++) == 3)
		shc = 0;
	set_indicator(numi / pow10[shc], shc);
		
	switch(decc) {
		default:
			decc++;
			break;
		case BUTT_TIME_TRIG:
			decc++;
			numi--;
			if(numi < 0)
				numi = 0;
			break;
		case BUTT_TIME_COMPLETE:
			break;
	}
	switch(incc) {
		default:
			incc++;
			break;
		case BUTT_TIME_TRIG:
			incc++;
			numi++;
			if(numi > 9999)
				numi = 9999;
			break;
		case BUTT_TIME_COMPLETE:
			break;
	}
	return;
}


void EXTI0_1_IRQHandler() {
	incc = BUTT_TIME_START;	
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);	
}

void EXTI4_15_IRQHandler() {
	decc = BUTT_TIME_START;
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
}



int main(void)
{
	rcc_config();
	gpio_config();
	
	exti_config();
	systick_config();
	
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);

	while(1);

	return 0;
}
