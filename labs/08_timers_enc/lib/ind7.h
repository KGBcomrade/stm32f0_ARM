#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"

#pragma once

void gpio_ind7_config() {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);


	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);	

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
}
	


void set_indicator(int num, int8_t sh) {
	
	
	static int32_t mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 \
				| LL_GPIO_PIN_4 | LL_GPIO_PIN_5 |  LL_GPIO_PIN_6 | LL_GPIO_PIN_7;

	static int32_t shmask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;


	static const uint32_t decoder[] = {
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
		LL_GPIO_PIN_4 | LL_GPIO_PIN_5, // 0
		LL_GPIO_PIN_1 | LL_GPIO_PIN_2, // 1
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | \
		LL_GPIO_PIN_3, // 2
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_6 | LL_GPIO_PIN_2 | \
		LL_GPIO_PIN_3, // 3
		LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2, // 4
		LL_GPIO_PIN_0 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_2 | \
			LL_GPIO_PIN_3, // 5
		LL_GPIO_PIN_0 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_2 | \
			LL_GPIO_PIN_3 | LL_GPIO_PIN_4, //6
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2, //7
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
			LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //8
		LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
			LL_GPIO_PIN_5 | LL_GPIO_PIN_6 //9
		
		
	};

	static const uint32_t shdecoder[] = {
		LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3
	};

	uint32_t port_state = LL_GPIO_ReadOutputPort(GPIOB);

	port_state = (port_state & ~mask) | decoder[(num & 0xF) % (sizeof(decoder) / sizeof(uint32_t))];
	if(num & 16)
		port_state |= LL_GPIO_PIN_7;
	LL_GPIO_WriteOutputPort(GPIOB, port_state);

	port_state = LL_GPIO_ReadOutputPort(GPIOC);
	port_state = (port_state & ~shmask) | (~shdecoder[sh] & shmask);
	LL_GPIO_WriteOutputPort(GPIOC, port_state);


} 
