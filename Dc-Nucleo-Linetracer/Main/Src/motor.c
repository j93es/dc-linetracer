/*
 * motor.c
 */

#include "header_init.h"





void MotorL_Start() {

	// motorL pwm start
	LL_TIM_EnableCounter(TIM10);
	LL_TIM_CC_EnableChannel(TIM10, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH2(TIM10, 0);

	// motorL encoder start
	LL_TIM_EnableCounter(TIM4);
}



void MotorR_Start() {

	// motorR pwm start
	LL_TIM_EnableCounter(TIM11);
	LL_TIM_CC_EnableChannel(TIM11, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH2(TIM11, 0);

	// motorR encoder start
	LL_TIM_EnableCounter(TIM3);
}




void Motor_Start() {

	MotorL_Start();
	MotorR_Start();
}





void MotorL_Power_Off() {

	TIM10->CCR1 = 0;
}


void MotorR_Power_Off() {

	TIM11->CCR1 = 0;
}



void Motor_Power_Off() {

	MotorL_Power_Off();
	MotorR_Power_Off();
}








void MotorL_Stop() {

	MotorL_Power_Off();

	// motorL pwm end
	LL_TIM_DisableCounter(TIM10);
	LL_TIM_CC_DisableChannel(TIM10, LL_TIM_CHANNEL_CH1);

	// motorL encoder end
	LL_TIM_DisableCounter(TIM4);
}



void MotorR_Stop() {

	MotorR_Power_Off();

	// motorR pwm end
	LL_TIM_DisableCounter(TIM11);
	LL_TIM_CC_DisableChannel(TIM11, LL_TIM_CHANNEL_CH1);

	// motorR encoder end
	LL_TIM_DisableCounter(TIM3);
}



void Motor_Stop() {

	MotorL_Stop();
	MotorR_Stop();
}







void Speed_Control_Start(){
	LL_TIM_EnableCounter(TIM9);
	LL_TIM_EnableIT_UPDATE(TIM9);
}




void Speed_Control_Stop(){
	LL_TIM_DisableIT_UPDATE(TIM9);
	LL_TIM_DisableCounter(TIM9);
}


