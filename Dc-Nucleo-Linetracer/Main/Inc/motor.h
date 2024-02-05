/*
 * motor.h
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_





#include "main.h"
#include "custom_gpio.h"
#include "sensor.h"
#include "drive_def_var.h"




void	MotorL_Start();
void	MotorR_Start();
void	Motor_Start();

void	MotorL_Stop();
void	MotorR_Stop();
void	Motor_Stop();

void	MotorL_Power_Off();
void	MotorR_Power_Off();
void	Motor_Power_Off();

void	Speed_Control_Start();
void	Speed_Control_Stop();







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
//
//	1바퀴 가는 데에 필요한 엔코더 값 = 2024

	targetEncoderValueL += TICK_PER_M * speed * MOTOR_CONTROL_INTERVAL_S;

	return targetEncoderValueL;
}


__STATIC_INLINE int32_t	Get_Current_Encoder_Value_L() {

	return TIM4->CNT;
}


__STATIC_INLINE float	Make_Target_Encoder_Value_R(float speed) {

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
//
//	1바퀴 가는 데에 필요한 엔코더 값 = 2024

	targetEncoderValueR += TICK_PER_M * speed * MOTOR_CONTROL_INTERVAL_S;

	return targetEncoderValueR;
}


__STATIC_INLINE int32_t	Get_Current_Encoder_Value_R() {

	return TIM3->CNT;
}








__STATIC_INLINE float	Make_P_Term(int32_t error) {

	float pTerm = error * pCoef;

	return pTerm;
}


__STATIC_INLINE int32_t	Make_Error_Diff(int32_t error, int32_t prevError, int32_t prevErrorDiff) {

	const float alpha = 0.5;
	int32_t	errorDiff = alpha * (error - prevError) + (1.f - alpha) * prevErrorDiff;

	return errorDiff;
}


__STATIC_INLINE float	Make_D_Term(int32_t errorDiff) {

	float dTerm = errorDiff * dCoef;

	return dTerm;
}


__STATIC_INLINE float	Get_Duty_Ratio(float pTerm, float dTerm) {

	float voltage = pTerm + dTerm;
	float dutyRatio = voltage / sensingVoltage;

	return dutyRatio;
}


__STATIC_INLINE uint32_t	Get_Level_CCR(float dutyRatio) {

	int32_t	levelCCR = ABS(dutyRatio * levelMaxCCR);

	if (levelCCR > levelMaxCCR) {
		levelCCR = levelMaxCCR;
	}
	else if (levelCCR < 0) {
		levelCCR = 0;
	}

	return levelCCR;
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








__STATIC_INLINE	void	Adjust_Encoder_Values_L() {

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


__STATIC_INLINE	void	Adjust_Encoder_Values_R() {

	uint32_t	curEncoderValueR = Get_Current_Encoder_Value_R();

	if (curEncoderValueR > ENCODER_VALUE_ADJUST_THRESHOLD_MAX) {

		int32_t	adjustValue = curEncoderValueR - ENCODER_VALUE_ADJUST_THRESHOLD_MID;
		targetEncoderValueR -= adjustValue;
		TIM3->CNT -= adjustValue;
	}
	else if (curEncoderValueR < ENCODER_VALUE_ADJUST_THRESHOLD_MIN) {
		int32_t	adjustValue = ENCODER_VALUE_ADJUST_THRESHOLD_MID - curEncoderValueR;
		targetEncoderValueR -= adjustValue;
		TIM3->CNT -= adjustValue;
	}
}










__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {

	int32_t	errorL = Make_Target_Encoder_Value_L(speedL) - Get_Current_Encoder_Value_L();
	int32_t errorR = Make_Target_Encoder_Value_R(speedR) - Get_Current_Encoder_Value_R();

	float pTermL = Make_P_Term(errorL);
	float pTermR = Make_P_Term(errorR);


	int32_t errorDiffL = Make_Error_Diff(errorL, prevErrorL, prevErrorDiffL);
	int32_t errorDiffR = Make_Error_Diff(errorR, prevErrorR, prevErrorDiffR);

	float dTermL = Make_D_Term(errorDiffL);
	float dTermR = Make_D_Term(errorDiffR);


	float	dutyRatioL = Get_Duty_Ratio(pTermL, dTermL);
	float	dutyRatioR = Get_Duty_Ratio(pTermR, dTermR);

	uint32_t levelCCR_L = Get_Level_CCR(dutyRatioL);
	uint32_t levelCCR_R = Get_Level_CCR(dutyRatioR);

	Motor_CCR_Control(levelCCR_L, levelCCR_R);
	Motor_Direction_Control(dutyRatioL, dutyRatioR);

	Adjust_Encoder_Values_L();
	Adjust_Encoder_Values_R();

	prevErrorL = errorL;
	prevErrorR = errorR;
	prevErrorDiffL = errorDiffL;
	prevErrorDiffR = errorDiffR;
}




__STATIC_INLINE void	Motor_Target_Control() {

	int32_t	errorL = targetEncoderValueL - Get_Current_Encoder_Value_L();
	int32_t	errorR = targetEncoderValueR - Get_Current_Encoder_Value_R();

	float pTermL = Make_P_Term(errorL);
	float pTermR = Make_P_Term(errorR);


	int32_t errorDiffL = Make_Error_Diff(errorL, prevErrorL, prevErrorDiffL);
	int32_t errorDiffR = Make_Error_Diff(errorR, prevErrorR, prevErrorDiffR);

	float dTermL = Make_D_Term(errorDiffL);
	float dTermR = Make_D_Term(errorDiffR);


	float	dutyRatioL = Get_Duty_Ratio(pTermL, dTermL);
	float	dutyRatioR = Get_Duty_Ratio(pTermR, dTermR);

	uint32_t levelCCR_L = Get_Level_CCR(dutyRatioL);
	uint32_t levelCCR_R = Get_Level_CCR(dutyRatioR);

	Motor_CCR_Control(levelCCR_L, levelCCR_R);
	Motor_Direction_Control(dutyRatioL, dutyRatioR);

	Adjust_Encoder_Values_L();
	Adjust_Encoder_Values_R();

	prevErrorL = errorL;
	prevErrorR = errorR;
	prevErrorDiffL = errorDiffL;
	prevErrorDiffR = errorDiffR;
}



//
//
//void	MotorL_Start();
//void	MotorR_Start();
//void	Motor_Start();
//
//void	MotorL_Stop();
//void	MotorR_Stop();
//void	Motor_Stop();
//
//void	MotorL_Power_Off();
//void	MotorR_Power_Off();
//void	Motor_Power_Off();
//
//void	Speed_Control_Start();
//void	Speed_Control_Stop();
//
//
//#define ENCODER_VALUE_ADJUST_THRESHOLD_MAX	57344
//#define ENCODER_VALUE_ADJUST_THRESHOLD_MID	32768
//#define ENCODER_VALUE_ADJUST_THRESHOLD_MIN	8192
//
//
//#define ENCODER_VALUE_PER_CIRCLE			2048
//
//
//#define PID_DELTA_T							0.0005
//
//
//
//
//__STATIC_INLINE float	Make_Target_Encoder_Value_L(float speed) {
//
////	작은 톱니 17개
////	큰 톱니 69개
////
////	바퀴 지름 3.6cm
////	3.6cm => 22.61946710cm
////
////	22.61946710cm 를 가려면 작은 톱니가 4.05588235 바퀴를 가야함
////
////	1cm 를 가려면 0.17943939667 바퀴 가야함
////	100cm 를 가려면 17.943939667 바퀴 가야함
////
////	1바퀴 가는 데에 필요한 엔코더 값 = 2024
//
//	targetEncoderValueL += ENCODER_VALUE_PER_CIRCLE * 17.944f * speed / 2000;
//
//	return targetEncoderValueL;
//}
//
//
//__STATIC_INLINE int32_t	Get_Current_Encoder_Value_L() {
//
//	return TIM4->CNT;
//}
//
//
//__STATIC_INLINE float	Get_Duty_Ratio_L(int32_t errorL) {
//
//	float	errorDiffL = errorL - prevErrorL;
//	float	voltageL = pCoef * errorL + dCoef * errorDiffL;
//	float	dutyRatioL = voltageL / GET_MAX(sensingVoltage, 1);
//
//	if (dutyRatioL > 1.f) {
//		dutyRatioL = 1.f;
//	} else if (dutyRatioL < -1.f) {
//		dutyRatioL = -1.f;
//	}
//
//	prevErrorL = errorL;
//
//	return dutyRatioL;
//}
//
//
//__STATIC_INLINE uint32_t	Get_Level_CCR_L(float dutyRatioL) {
//
//	uint32_t	levelCCR_L = ABS(dutyRatioL * levelMaxCCR_L);
//
//	if (levelCCR_L > levelMaxCCR_L) {
//		levelCCR_L = levelMaxCCR_L;
//	}
//
//	return levelCCR_L;
//}
//
//
//
//__STATIC_INLINE	void	Adjust_Encoder_Values_L() {
//
//	uint32_t	curEncoderValueL = Get_Current_Encoder_Value_L();
//
//	if (curEncoderValueL > ENCODER_VALUE_ADJUST_THRESHOLD_MAX) {
//
//		int32_t	adjustValue = curEncoderValueL - ENCODER_VALUE_ADJUST_THRESHOLD_MID;
//		targetEncoderValueL -= adjustValue;
//		TIM4->CNT -= adjustValue;
//	}
//	else if (curEncoderValueL < ENCODER_VALUE_ADJUST_THRESHOLD_MIN) {
//		int32_t	adjustValue = ENCODER_VALUE_ADJUST_THRESHOLD_MID - curEncoderValueL;
//		targetEncoderValueL -= adjustValue;
//		TIM4->CNT -= adjustValue;
//	}
//}
//
//
//
//
//
//
//
//
//
//
//__STATIC_INLINE float	Make_Target_Encoder_Value_R(float speedR) {
//
////	작은 톱니 17개
////	큰 톱니 69개
////
////	바퀴 지름 3.6cm
////	3.6cm => 22.61946710cm
////
////	22.61946710cm 를 가려면 작은 톱니가 4.05588235 바퀴를 가야함
////
////	1cm 를 가려면 0.17943939667 바퀴 가야함
////	100cm 를 가려면 17.943939667 바퀴 가야함
//
//	targetEncoderValueR += ENCODER_VALUE_PER_CIRCLE * 17.944f * speedR / 2000;
//
//	return targetEncoderValueR;
//}
//
//
//__STATIC_INLINE int32_t	Get_Current_Encoder_Value_R() {
//
//	return TIM3->CNT;
//}
//
//
//__STATIC_INLINE float	Get_Duty_Ratio_R(int32_t errorR) {
//
//	float	errorDiffR = errorR - prevErrorR;
//	float	voltageR = pCoef * errorR + dCoef * errorDiffR;
//	float	dutyRatioR = voltageR / GET_MAX(sensingVoltage, 1);
//
//	if (dutyRatioR > 1.f) {
//		dutyRatioR = 1.f;
//	} else if (dutyRatioR < -1.f) {
//		dutyRatioR = -1.f;
//	}
//
//	prevErrorR = errorR;
//
//	return dutyRatioR;
//}
//
//
//__STATIC_INLINE uint32_t	Get_Level_CCR_R(float dutyRatioR) {
//
//	uint32_t	levelCCR_R = ABS(dutyRatioR * levelMaxCCR_R);
//
//	if (levelCCR_R > levelMaxCCR_R) {
//		levelCCR_R = levelMaxCCR_R;
//	}
//
//	return levelCCR_R;
//}
//
//
//__STATIC_INLINE	void	Adjust_Encoder_Values_R() {
//
//	uint32_t	curEncoderValueR = Get_Current_Encoder_Value_R();
//
//	if (curEncoderValueR > ENCODER_VALUE_ADJUST_THRESHOLD_MAX) {
//
//		int32_t	adjustValue = curEncoderValueR - ENCODER_VALUE_ADJUST_THRESHOLD_MID;
//		targetEncoderValueR -= adjustValue;
//		TIM3->CNT -= adjustValue;
//	}
//	else if (curEncoderValueR < ENCODER_VALUE_ADJUST_THRESHOLD_MIN) {
//		int32_t	adjustValue = ENCODER_VALUE_ADJUST_THRESHOLD_MID - curEncoderValueR;
//		targetEncoderValueR -= adjustValue;
//		TIM3->CNT -= adjustValue;
//	}
//}
//
//
//
//
//
//
//
//__STATIC_INLINE void	Motor_CCR_Control(uint32_t levelCCR_L, uint32_t levelCCR_R) {
//
//	TIM10->CCR1 = levelCCR_L;
//	TIM11->CCR1 = levelCCR_R;
//}
//
//
//
//__STATIC_INLINE void	Motor_Direction_Control(float dutyRatioL, float dutyRatioR) {
//	Custom_GPIO_Set(GPIOB, 1 << 4, dutyRatioL > 0 ? 1 : 0); // PB4
//	Custom_GPIO_Set(GPIOB, 1 << 5, dutyRatioL < 0 ? 1 : 0); // PB5
//
//	Custom_GPIO_Set(GPIOC, 1 << 4, dutyRatioR < 0 ? 1 : 0); // PC4
//	Custom_GPIO_Set(GPIOC, 1 << 5, dutyRatioR > 0 ? 1 : 0); // PC5
//}
//
//
//
//
//
//
//
//
//
//
//
//
//__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {
//
//	int32_t	errorL = Make_Target_Encoder_Value_L(speedL) - Get_Current_Encoder_Value_L();
//	int32_t errorR = Make_Target_Encoder_Value_R(speedR) - Get_Current_Encoder_Value_R();
//
//	float	dutyRatioL = Get_Duty_Ratio_L(errorL);
//	float	dutyRatioR = Get_Duty_Ratio_R(errorR);
//
//	uint32_t levelCCR_L = Get_Level_CCR_L(dutyRatioL);
//	uint32_t levelCCR_R = Get_Level_CCR_R(dutyRatioR);
//
//	Motor_CCR_Control(levelCCR_L, levelCCR_R);
//	Motor_Direction_Control(dutyRatioL, dutyRatioR);
//
//	Adjust_Encoder_Values_L();
//	Adjust_Encoder_Values_R();
//}
//
//
//
//
//__STATIC_INLINE void	Motor_Target_Control() {
//
//	int32_t	errorL = targetEncoderValueL - Get_Current_Encoder_Value_L();
//	int32_t	errorR = targetEncoderValueR - Get_Current_Encoder_Value_R();
//
//	float	dutyRatioL = Get_Duty_Ratio_L(errorL);
//	float	dutyRatioR = Get_Duty_Ratio_R(errorR);
//
//	uint32_t levelCCR_L = Get_Level_CCR_L(dutyRatioL);
//	uint32_t levelCCR_R = Get_Level_CCR_R(dutyRatioR);
//
//	Motor_CCR_Control(levelCCR_L, levelCCR_R);
//	Motor_Direction_Control(dutyRatioL, dutyRatioR);
//
//	Adjust_Encoder_Values_L();
//	Adjust_Encoder_Values_R();
//}

#endif /* INC_MOTOR_H_ */
