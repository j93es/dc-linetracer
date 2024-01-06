/*
 * custom_gpio.h
 *
 *  Created on: Apr 13, 2021
 *      Author: Joonho Gwon
 */

#ifndef INC_CUSTOM_GPIO_H_
#define INC_CUSTOM_GPIO_H_

#include "stm32f4xx.h"

typedef volatile struct {
	GPIO_TypeDef *port;
	uint32_t pinMask;
} Custom_GPIO_t;

/*
 * 아래 Custom_GPIO_Set 함수 등을 .c 파일에 구현하지 않고 헤더 파일에 static으로 구현하였다.
 * 이는 C언어에서 inline function이 일반 함수와 다르게 취급되기 때문으로, 자세한 이유는 본인도 잘 모르므로 생략한다.
 */

static inline void Custom_GPIO_Set(GPIO_TypeDef *GPIOx, uint32_t PinMask,
		uint32_t value) {

	/*
	 * 이 함수는 LL_GPIO~~ 함수 대신 복잡한 레지스터를 사용한다.
	 * 이렇게 하는 이유는 if문을 최대한 사용하지 않기 위해서다.
	 * CPU 내부에는 파이프라이닝이라는 절차가 있어서, 한 명령어를 실행하면서 동시에 다음 명령어를 로드해온다.
	 * 그런데 if문이 있으면 그 조건에 따라 다음 명령어가 무엇인지가 달라지므로 CPU는 다음 명령어를 정확히 알지 못하고 예측하여 로드하는데 이를 분기 예측이라 한다.
	 * 이 분기 예측이 틀릴 경우 로드해둔 명령어는 전혀 쓸 수 없게 되므로 CPU에서는 수 클럭 이상을 낭비한다.
	 * Custom_GPIO_Set 함수와 같은 경우 분기 예측이 틀릴 가능성이 매우 높은 함수다.
	 * 따라서 분기 예측을 할 수 있도록 if문 없이 비트 연산만을 사용하여 GPIO 설정을 구현한 것이다.
	 */

	GPIOx->ODR = (GPIOx->ODR & ~PinMask)
			| (PinMask & (((!!value) << 31) >> 31));

#if 0

	/*
	 * 아래 구문은 위와 동일한 역할을 하지만, 파이프라이닝 분기 예측 실패 시 속도가 조금 더 느려질 수 있다.
	 * 사실 그 속도 차이가 크게 유의미하지는 않을 수 있다만...
	 */

	if(value){
		LL_GPIO_SetOutputPin(GPIOx, PinMask);
	}else{
		LL_GPIO_ResetOutputPin(GPIOx, PinMask);
	}
#endif
}

static inline void Custom_GPIO_Set_t(Custom_GPIO_t *Pin, uint32_t value) {
	Pin->port->ODR = (Pin->port->ODR & ~Pin->pinMask)
			| (Pin->pinMask & (((!!value) << 31) >> 31));
}

#endif /* INC_CUSTOM_GPIO_H_ */
