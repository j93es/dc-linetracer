/*
 * motor.h
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include <config.h>
#include "main.h"
#include "custom_gpio.h"
#include "sensor.h"




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





//
//__STATIC_INLINE int32_t	Get_Encoder_Value_Gap(t_encoder target, t_encoder current) {
//
//	int32_t gap = target - current;
//	int32_t absGap = ABS(gap);
//
//	if (absGap > T_ENCODER_MAX / 2) {
//
//		gap = (gap > 0 ? -1 : 1) * (T_ENCODER_MAX - absGap);
//	}
//
//	return gap;
//}
//
//
//
//
//
//__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {
//
//
//	t_encoder curEncoderValueL = TIM4->CNT;
//	t_encoder curEncoderValueR = TIM3->CNT;
//
//	targetEncoderValueL_cntl += speedL * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;
//	targetEncoderValueR_cntl += speedR * TICK_PER_M * MOTOR_CONTROL_INTERVAL_S;
//
//
//	int32_t	errorL = Get_Encoder_Value_Gap(targetEncoderValueL_cntl, curEncoderValueL);
//	int32_t errorR = Get_Encoder_Value_Gap(targetEncoderValueR_cntl, curEncoderValueR);
//
//	float pTermL = errorL * pCoef;
//	float pTermR = errorR * pCoef;
//
//	float dTermL = (errorL - prevErrorL) * dCoef;
//	float dTermR = (errorR - prevErrorR) * dCoef;
//
//
//	float curVoltage = sensingVoltage + valocityL * MOTOR_KE;
//	float dutyRatioL = (pTermL + dTermL) / curVoltage;
//	float dutyRatioR = (pTermR + dTermR) / curVoltage;
//
//
//	uint32_t levelCCR_L = ABS(dutyRatioL * levelMaxCCR);
//	levelCCR_L = GET_MIN(levelCCR_L, levelMaxCCR);
//
//	uint32_t levelCCR_R = ABS(dutyRatioR * levelMaxCCR);
//	levelCCR_R = GET_MIN(levelCCR_R, levelMaxCCR);
//
//
//	TIM10->CCR1 = levelCCR_L;
//	TIM11->CCR1 = levelCCR_R;
//
//
//	Custom_GPIO_Set(GPIOB, 1 << 4, dutyRatioL > 0 ? 1 : 0); // PB4
//	Custom_GPIO_Set(GPIOB, 1 << 5, dutyRatioL < 0 ? 1 : 0); // PB5
//
//	Custom_GPIO_Set(GPIOC, 1 << 4, dutyRatioR < 0 ? 1 : 0); // PC4
//	Custom_GPIO_Set(GPIOC, 1 << 5, dutyRatioR > 0 ? 1 : 0); // PC5
//
//
//	curTick_L += Get_Encoder_Value_Gap(curEncoderValueL, prevEncoderValueL);
//	curTick_R += Get_Encoder_Value_Gap(curEncoderValueR, prevEncoderValueR);
//
//	prevErrorL = errorL;
//	prevErrorR = errorR;
//	prevEncoderValueL = curEncoderValueL;
//	prevEncoderValueR = curEncoderValueR;
//
//}
//
//

//__STATIC_INLINE int32_t	Get_Encoder_Value_Gap(t_encoder target, t_encoder current) {
//
//	int32_t gap = target - current;
//	int32_t absGap = ABS(gap);
//
//	if (absGap > T_ENCODER_MAX / 2) {
//
//		gap = (gap > 0 ? -1 : 1) * (T_ENCODER_MAX - absGap);
//	}
//
//	return gap;
//}



__STATIC_INLINE void	Motor_Speed_Control(float speedL, float speedR) {

	float curVoltage = sensingVoltage;

	t_encoder curEncoderValueL = TIM4->CNT;
	t_encoder curEncoderValueR = TIM3->CNT;

	float velocityCmdL = RADIAN_PER_M * speedL;
	float velocityCmdR = RADIAN_PER_M * speedR;

	positionCmdL += velocityCmdL * MOTOR_CONTROL_INTERVAL_S;
	positionCmdR += velocityCmdR * MOTOR_CONTROL_INTERVAL_S;

	t_encoder deltaTickL = (t_encoder)(curEncoderValueL - prevEncoderValueL);
	t_encoder deltaTickR = (t_encoder)(curEncoderValueR - prevEncoderValueR);

	float velocityL = deltaTickL * RADIAN_PER_TICK / MOTOR_CONTROL_INTERVAL_S;
	float velocityR = deltaTickR * RADIAN_PER_TICK / MOTOR_CONTROL_INTERVAL_S;

	positionL += velocityL * MOTOR_CONTROL_INTERVAL_S;
	positionR += velocityR * MOTOR_CONTROL_INTERVAL_S;

	// anti windup
	positionCmdL = GET_MIN(positionCmdL, positionL + 0.5f);
	positionCmdL = GET_MAX(positionCmdL, positionL - 0.5f);

	positionCmdR = GET_MIN(positionCmdR, positionR + 0.5f);
	positionCmdR = GET_MAX(positionCmdR, positionR - 0.5f);


	float positionErrL = positionL - positionCmdL;
	float positionErrR = positionR - positionCmdR;

	float velocityErrL = velocityL - velocityCmdL;
	float velocityErrR = velocityR - velocityCmdR;

	float currentL = -(positionErrL * pCoef + velocityErrL * dCoef);
	float currentR = -(positionErrR * pCoef + velocityErrR * dCoef);

	float voltL = currentL * MOTOR_RESISTANCE + velocityL * MOTOR_KE;
	float voltR = currentR * MOTOR_RESISTANCE + velocityR * MOTOR_KE;

	float dutyRatioL = voltL / curVoltage;
	float dutyRatioR = voltR / curVoltage;

	uint32_t levelCCR_L = ABS(dutyRatioL) * levelMaxCCR;
	levelCCR_L = GET_MIN(levelCCR_L, levelMaxCCR);

	uint32_t levelCCR_R = ABS(dutyRatioR) * levelMaxCCR;
	levelCCR_R = GET_MIN(levelCCR_R, levelMaxCCR);

	TIM10->CCR1 = levelCCR_L;
	TIM11->CCR1 = levelCCR_R;


	Custom_GPIO_Set(GPIOB, 1 << 4, dutyRatioL > 0 ? 1 : 0); // PB4
	Custom_GPIO_Set(GPIOB, 1 << 5, dutyRatioL < 0 ? 1 : 0); // PB5

	Custom_GPIO_Set(GPIOC, 1 << 4, dutyRatioR < 0 ? 1 : 0); // PC4
	Custom_GPIO_Set(GPIOC, 1 << 5, dutyRatioR > 0 ? 1 : 0); // PC5


	curTick_L += deltaTickL;
	curTick_R += deltaTickR;
//	curMarkSamplingTick += (float)(deltaTickL + deltaTickR) / 2.f;

	prevEncoderValueL = curEncoderValueL;
	prevEncoderValueR = curEncoderValueR;

}





#endif /* INC_MOTOR_H_ */
