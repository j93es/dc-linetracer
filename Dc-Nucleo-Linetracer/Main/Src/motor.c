/*
 * motor.c
 */

#include "motor.h"

#include "main.h"
#include "custom_gpio.h"



volatile Custom_GPIO_t	motorL[4];

volatile Custom_GPIO_t	motorR[4];




volatile uint8_t	phaseL_table[8] = { 0x09, 0x08, 0x0C, 0x04, 0x06, 0x02, 0x03, 0x01 };
volatile uint8_t	phaseR_table[8] = { 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09 };

volatile uint8_t	phaseL  = 0;
volatile uint8_t	phaseR  = 0;




void Motor_Power_Off() {

	Custom_GPIO_Set_t(motorL + 0, 0);
	Custom_GPIO_Set_t(motorL + 1, 0);
	Custom_GPIO_Set_t(motorL + 2, 0);
	Custom_GPIO_Set_t(motorL + 3, 0);

	Custom_GPIO_Set_t(motorR + 0, 0);
	Custom_GPIO_Set_t(motorR + 1, 0);
	Custom_GPIO_Set_t(motorR + 2, 0);
	Custom_GPIO_Set_t(motorR + 3, 0);
}





void Motor_Start() {
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);

	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableIT_UPDATE(TIM4);
}





void Motor_Stop() {
	Motor_Power_Off();

	LL_TIM_DisableIT_UPDATE(TIM3);
	LL_TIM_DisableCounter(TIM3);

	LL_TIM_DisableIT_UPDATE(TIM4);
	LL_TIM_DisableCounter(TIM4);

	Motor_Power_Off();

	curTick_L = 0;
	curTick_R = 0;
}




void Speed_Control_Start(){
	LL_TIM_EnableCounter(TIM9);
	LL_TIM_EnableIT_UPDATE(TIM9);
}




void Speed_Control_Stop(){
	LL_TIM_DisableIT_UPDATE(TIM9);
	LL_TIM_DisableCounter(TIM9);
}


