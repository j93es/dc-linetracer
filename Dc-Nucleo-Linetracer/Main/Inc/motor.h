/*
 * motor.h
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_





#include "main.h"
#include "custom_gpio.h"
#include "sensor.h"
#include "drive_def_var.h"




void	Motor_Power_Off();
void	Motor_Start();
void	Motor_Stop();

void	Speed_Control_Start();
void	Speed_Control_Stop();


#define ENCODER_VALUE_ADJUST_THRESHOLD_MAX	49152
#define ENCODER_VALUE_ADJUST_THRESHOLD_MID	32768
#define ENCODER_VALUE_ADJUST_THRESHOLD_MIN	16384




__STATIC_INLINE float	Make_Target_Encoder_Value_L(float speed) {

//	작은 톱니 17개
//	큰 톱니 69개
//
//	바퀴 지름 3.6cm
//	3.6cm => 22.61946710cm
//
//	22.61946710cm 를 가려면 작은 톱니가 4.05588235 바퀴를 가야함
//
//	1cm 를 가려면 0.17943939667 바퀴 가야함
//	100cm 를 가려면 17.943939667 바퀴 가야함

	targetEncoderValueL += 256 * 17.944 * speed / 2000;

	return targetEncoderValueL;
}


__STATIC_INLINE int32_t	Get_Current_Encoder_Value_L() {

	return TIM4->CNT;
}


__STATIC_INLINE float	Get_Duty_Ratio_L(int32_t errorL) {

	float	errorDiffL = errorL - prevErrorL;
	float	voltageL = pCoef * errorL + dCoef * errorDiffL;
	float	dutyRatioL = voltageL / sensingVoltage;

	prevErrorL = errorL;

	return dutyRatioL;
}


__STATIC_INLINE uint32_t	Get_Level_CCR_L(float dutyRatioL) {

	uint32_t	levelCCR_L = ABS(dutyRatioL * levelMaxCCR_L);

	if (levelCCR_L > levelMaxCCR_L) {
		levelCCR_L = levelMaxCCR_L;
	}

	return levelCCR_L;
}






__STATIC_INLINE void	Motor_CCR_Control(uint32_t levelCCR_L, uint32_t levelCCR_R) {

	TIM10->CCR1 = levelCCR_L;
	TIM11->CCR1 = levelCCR_R;
}


__STATIC_INLINE void	Motor_Direction_Control(float dutyRatioL, float dutyRatioR) {
	Custom_GPIO_Set(GPIOB, 1 << 4, dutyRatioL > 0 ? 1 : 0); // PB4
	Custom_GPIO_Set(GPIOB, 1 << 5, dutyRatioL < 0 ? 1 : 0); // PB5

	Custom_GPIO_Set(GPIOC, 1 << 4, dutyRatioR < 0 ? 1 : 0); // PC4
	Custom_GPIO_Set(GPIOC, 1 << 5, dutyRatioR > 0 ? 1 : 0); // PC5
}






__STATIC_INLINE	void	Adjust_Encder_Values_L() {

	uint32_t	curEncoderValueL = Get_Current_Encoder_Value_L();

	if (curEncoderValueL > ENCODER_VALUE_ADJUST_THRESHOLD_MAX) {

		int32_t	adjustValue = curEncoderValueL - ENCODER_VALUE_ADJUST_THRESHOLD_MID;
		targetEncoderValueL -= adjustValue;
		TIM4->CNT -= adjustValue;
	}
	else if (curEncoderValueL < ENCODER_VALUE_ADJUST_THRESHOLD_MIN) {
		int32_t	adjustValue = ENCODER_VALUE_ADJUST_THRESHOLD_MID - curEncoderValueL;
		targetEncoderValueL -= adjustValue;
		TIM4->CNT -= adjustValue;
	}
}




__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {

	int32_t	errorL = Make_Target_Encoder_Value_L(speedL) - Get_Current_Encoder_Value_L();
//	int32_t errorR = Make_Target_Encoder_Value_R(speedR) - Get_Current_Encoder_Value_R();

	float	dutyRatioL = Get_Duty_Ratio_L(errorL);
//	float	dutyRatioR = Get_Duty_Ratio_R(errorR);

	uint32_t levelCCR_L = Get_Level_CCR_L(dutyRatioL);
//	uint32_t levelCCR_R = Get_Level_CCR_R(dutyRatioR);

	Motor_CCR_Control(levelCCR_L, levelCCR_L);	// 추후 L을 R로 변경
	Motor_Direction_Control(dutyRatioL, dutyRatioL);// 추후 L을 R로 변경

	Adjust_Encder_Values_L();
}




#endif /* INC_MOTOR_H_ */
