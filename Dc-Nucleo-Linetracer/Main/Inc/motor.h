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






__STATIC_INLINE int32_t	Get_Encoder_Value_Gap(t_encoder target, t_encoder current) {

	int32_t gap = target - current;
	int32_t absGap = ABS(gap);

	if (absGap > T_ENCODER_MAX / 2) {

		gap = (gap > 0 ? -1 : 1) * (T_ENCODER_MAX - absGap);
	}

	return gap;
}





__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {


	t_encoder curEncoderValueL = TIM4->CNT;
	t_encoder curEncoderValueR = TIM3->CNT;

	targetEncoderValueL_cntl += speedL * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;
	targetEncoderValueR_cntl += speedR * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;


	int32_t	errorL = Get_Encoder_Value_Gap(targetEncoderValueL_cntl, curEncoderValueL);
	int32_t errorR = Get_Encoder_Value_Gap(targetEncoderValueR_cntl, curEncoderValueR);

	float pTermL = errorL * pCoef;
	float pTermR = errorR * pCoef;

	float dTermL = (errorL - prevErrorL) * dCoef;
	float dTermR = (errorR - prevErrorR) * dCoef;


	float curVoltage = sensingVoltage;
	float dutyRatioL = (pTermL + dTermL) / curVoltage;
	float dutyRatioR = (pTermR + dTermR) / curVoltage;


	uint32_t levelCCR_L = ABS(dutyRatioL * levelMaxCCR);
	levelCCR_L = GET_MIN(levelCCR_L, levelMaxCCR);

	uint32_t levelCCR_R = ABS(dutyRatioR * levelMaxCCR);
	levelCCR_R = GET_MIN(levelCCR_R, levelMaxCCR);


	TIM10->CCR1 = levelCCR_L;
	TIM11->CCR1 = levelCCR_R;


	Custom_GPIO_Set(GPIOB, 1 << 4, dutyRatioL > 0 ? 1 : 0); // PB4
	Custom_GPIO_Set(GPIOB, 1 << 5, dutyRatioL < 0 ? 1 : 0); // PB5

	Custom_GPIO_Set(GPIOC, 1 << 4, dutyRatioR < 0 ? 1 : 0); // PC4
	Custom_GPIO_Set(GPIOC, 1 << 5, dutyRatioR > 0 ? 1 : 0); // PC5


	curTick_L += Get_Encoder_Value_Gap(curEncoderValueL, prevCurEncoderValueL);
	curTick_R += Get_Encoder_Value_Gap(curEncoderValueR, prevCurEncoderValueR);

	prevErrorL = errorL;
	prevErrorR = errorR;
	prevCurEncoderValueL = curEncoderValueL;
	prevCurEncoderValueR = curEncoderValueR;

}



//
//
//
//
//__STATIC_INLINE t_encoder	Make_Target_Encoder_Value_L(float speed) {
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
//	targetEncoderValueL_cntl += speed * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;
//
//	return targetEncoderValueL_cntl;
//}
//
//
//__STATIC_INLINE t_encoder	Get_Current_Encoder_Value_L() {
//
//	return (t_encoder)TIM4->CNT;
//}
//
//
//
//
//
//
//__STATIC_INLINE t_encoder	Make_Target_Encoder_Value_R(float speed) {
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
//	targetEncoderValueR_cntl += speed * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;
//
//	return targetEncoderValueR_cntl;
//}
//
//
//__STATIC_INLINE t_encoder	Get_Current_Encoder_Value_R() {
//
//	return (t_encoder)TIM3->CNT;
//}
//
//
//
//
//
//
//__STATIC_INLINE float	Make_P_Term(int32_t error) {
//
//	float pTerm = error * pCoef;
//
//	return pTerm;
//}
//
//
//
//__STATIC_INLINE int32_t	Make_Error_Diff(int32_t error, int32_t prevError, int32_t prevErrorDiff) {
//
////	const float alpha = 0.5;
////	int32_t	errorDiff = alpha * (error - prevError) + (1.f - alpha) * prevErrorDiff;
//	int32_t	errorDiff = error - prevError;
//
//	return errorDiff;
//}
//
//
//__STATIC_INLINE float	Make_D_Term(int32_t errorDiff) {
//
//	float dTerm = errorDiff * dCoef;
//
//	return dTerm;
//}
//
//
//__STATIC_INLINE float	Get_Duty_Ratio(float pTerm, float dTerm) {
//
//	float voltage = pTerm + dTerm;
//	float dutyRatio = voltage / GET_MAX(sensingVoltage, 1);
//
//	return dutyRatio;
//}
//
//
//__STATIC_INLINE uint32_t	Get_Level_CCR(float dutyRatio) {
//
//	uint32_t	levelCCR = ABS(dutyRatio * levelMaxCCR);
//
//	if (levelCCR > levelMaxCCR) {
//		levelCCR = levelMaxCCR;
//	}
//
//	return levelCCR;
//}
//
//
//__STATIC_INLINE void	Motor_CCR_Control(uint32_t levelCCR_L, uint32_t levelCCR_R) {
//
//	TIM10->CCR1 = levelCCR_L;
//	TIM11->CCR1 = levelCCR_R;
//}
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
//__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {
//
//	t_encoder curEncoderValueL = Get_Current_Encoder_Value_L();
//	t_encoder curEncoderValueR = Get_Current_Encoder_Value_R();
//
//	t_encoder targetEncoderValueL = Make_Target_Encoder_Value_L(speedL);
//	t_encoder targetEncoderValueR = Make_Target_Encoder_Value_R(speedR);
//
//	int32_t	errorL = Get_Encoder_Value_Gap(targetEncoderValueL, curEncoderValueL);
//	int32_t errorR = Get_Encoder_Value_Gap(targetEncoderValueR, curEncoderValueR);
//
//	float pTermL = Make_P_Term(errorL);
//	float pTermR = Make_P_Term(errorR);
//
//
//	int32_t errorDiffL = Make_Error_Diff(errorL, prevErrorL, prevErrorDiffL);
//	int32_t errorDiffR = Make_Error_Diff(errorR, prevErrorR, prevErrorDiffR);
//
//	float dTermL = Make_D_Term(errorDiffL);
//	float dTermR = Make_D_Term(errorDiffR);
//
//
//	float	dutyRatioL = Get_Duty_Ratio(pTermL, dTermL);
//	float	dutyRatioR = Get_Duty_Ratio(pTermR, dTermR);
//
//	uint32_t levelCCR_L = Get_Level_CCR(dutyRatioL);
//	uint32_t levelCCR_R = Get_Level_CCR(dutyRatioR);
//
//	Motor_CCR_Control(levelCCR_L, levelCCR_R);
//	Motor_Direction_Control(dutyRatioL, dutyRatioR);
//
//	curTick_L += Get_Encoder_Value_Gap(curEncoderValueL, prevCurEncoderValueL);
//	curTick_R += Get_Encoder_Value_Gap(curEncoderValueR, prevCurEncoderValueR);
//
//	prevErrorL = errorL;
//	prevErrorR = errorR;
//	prevErrorDiffL = errorDiffL;
//	prevErrorDiffR = errorDiffR;
//	prevCurEncoderValueL = curEncoderValueL;
//	prevCurEncoderValueR = curEncoderValueR;
//
//}




#endif /* INC_MOTOR_H_ */
