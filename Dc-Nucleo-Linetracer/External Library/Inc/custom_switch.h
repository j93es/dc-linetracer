/*
 ******************************************************************************
 * file			custom_switch.h
 * author		Joonho Gwon
 * brief		Switch input
 ******************************************************************************
 */

#ifndef INC_CUSTOM_SWITCH_H_
#define INC_CUSTOM_SWITCH_H_

#define CUSTOM_SW_NONE	0x00
#define CUSTOM_SW_1		0x01
#define CUSTOM_SW_2		0x02
#define CUSTOM_SW_3		0x04
#define CUSTOM_SW_1_2	0x03
#define CUSTOM_SW_1_3	0x05
#define CUSTOM_SW_2_3	0x06
#define CUSTOM_SW_ALL	0x07


#include "stm32f4xx_ll_gpio.h"
#include "custom_delay.h"
#include <stdint.h>
#include <stdbool.h>

uint8_t Custom_Switch_Read(void);

/*
 ******************************************************************************
 * User Settings
 *
 * SW1 : Left switch
 * SW2 : Right swicth
 ******************************************************************************
 */

#define SW1_PORT	GPIOC
#define SW1_PIN		LL_GPIO_PIN_10
#define SW2_PORT	GPIOC
#define SW2_PIN		LL_GPIO_PIN_11
#define SW3_PORT	GPIOC
#define SW3_PIN		LL_GPIO_PIN_12


#endif /* INC_CUSTOM_SWITCH_H_ */
