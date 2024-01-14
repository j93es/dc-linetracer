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


	phaseL = (phaseL + 1) & 0x07;


	// tick 값 증가
	curTick_L += 1;
}





__STATIC_INLINE void	Motor_R_TIM4_IRQ() {


	phaseR = (phaseR + 1) & 0x07;


	// tick 값 증가
	curTick_R += 1;
}





#endif /* INC_MOTOR_H_ */
