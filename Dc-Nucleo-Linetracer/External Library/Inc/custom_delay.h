/*
 * custom-delay.h
 *
 *  Created on: Apr 10, 2021
 *      Author: Joonho Gwon
 *    Modifier: Pierre de Starlit
 */

#ifndef CUSTOM_DELAY_CUSTOM_DELAY_H_
#define CUSTOM_DELAY_CUSTOM_DELAY_H_

#include <stdint.h>
#include "stm32f4xx_ll_tim.h"

#define CUSTOM_DELAY_TIM TIM2
//전원을 켜고 난 뒤 경과한 시간을 인터럽트를 통해 측정한다.
//아래 변수는 인터럽트에 정의되어 있다.
extern volatile uint32_t uwTick;

__STATIC_INLINE void Custom_Delay_us(uint32_t microsecond) {
	CUSTOM_DELAY_TIM->ARR = microsecond - 1;
	CUSTOM_DELAY_TIM->SR &= ~(0x0001); // Clear UEV flag
	CUSTOM_DELAY_TIM->CR1 |= 1UL;
	while ((CUSTOM_DELAY_TIM->SR & 0x0001) != 1)
		;
}

__STATIC_INLINE void Custom_Delay_ms(uint32_t millisecond) {
	Custom_Delay_us(millisecond * 1000);
}

/**
 * @brief Systick을 초기화하여 1 ms당 시간 값(uwTick)을 1씩 증가하도록 하는 함수.
 *        프로그램을 시작하면 메뉴를 실행하기 전에 한 번만 실행한다.
 *        TIM2 사용도 검토를 했었으나 타이머 주기가 1us인 만큼 이 타이머의 인터럽트를 그대로 사용하면 성능이 현저히 떨어질 수 있어
 *        SysTick 사용을 하는 것으로 결정.
 *        이미 Custom_Delay 함수를 사용한 학생들도 있어서 Custom_Delay 함수는 그대로 유지한다.
 */
__STATIC_INLINE void Custom_Delay_Init_SysTick() {
	uwTick = 0;

	// 100: Clock Source
	// 010: TICKINT
	// 001: ENABLE
	SysTick->CTRL = (SysTick->CTRL & 0xFFFFFFF8) | 0x07;
}

/**
 * @brief SysTick이 초기화된 시점부터 경과한 시간을 반환한다.
 */
__STATIC_INLINE uint32_t Custom_Delay_Get_SysTick() {
	return uwTick;
}

#endif /* CUSTOM_DELAY_CUSTOM_DELAY_H_ */
