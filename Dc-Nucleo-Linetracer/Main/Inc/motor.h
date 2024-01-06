/*
 * motor.h
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_





#include "main.h"
#include "custom_gpio.h"




void	Motor_Power_Off();
void	Motor_Start();
void	Motor_Stop();

void	Speed_Control_Start();
void	Speed_Control_Stop();


extern volatile Custom_GPIO_t	motorL[4];
extern volatile Custom_GPIO_t	motorR[4];

extern volatile uint8_t			phaseL_table[8];
extern volatile uint8_t			phaseR_table[8];

extern volatile uint8_t			phaseL;
extern volatile uint8_t			phaseR;


// 현재 모터에 몇번 상이 잡혔는지를 카운트하는 변수
extern volatile uint32_t	curTick_L;
extern volatile uint32_t	curTick_R;




__STATIC_INLINE void	Motor_L_TIM3_IRQ() {

	// motorL phase 잡기
	Custom_GPIO_Set_t(motorL + 0, (phaseL_table[phaseL] >> 0) & 0x01);
	Custom_GPIO_Set_t(motorL + 1, (phaseL_table[phaseL] >> 1) & 0x01);
	Custom_GPIO_Set_t(motorL + 2, (phaseL_table[phaseL] >> 2) & 0x01);
	Custom_GPIO_Set_t(motorL + 3, (phaseL_table[phaseL] >> 3) & 0x01);

	phaseL = (phaseL + 1) & 0x07;


	// tick 값 증가
	curTick_L += 1;
}





__STATIC_INLINE void	Motor_R_TIM4_IRQ() {

	// motorR phase 잡기
	Custom_GPIO_Set_t(motorR + 0, (phaseR_table[phaseR] >> 0) & 0x01);
	Custom_GPIO_Set_t(motorR + 1, (phaseR_table[phaseR] >> 1) & 0x01);
	Custom_GPIO_Set_t(motorR + 2, (phaseR_table[phaseR] >> 2) & 0x01);
	Custom_GPIO_Set_t(motorR + 3, (phaseR_table[phaseR] >> 3) & 0x01);

	phaseR = (phaseR + 1) & 0x07;


	// tick 값 증가
	curTick_R += 1;
}





#endif /* INC_MOTOR_H_ */
