/*
 * custom_switch.h
 *
 * Modified on: Jul 7, 2023
 *      Author: Joonho Gwon
 *    Modifier: Pierre de Starlit
 */

#include "custom_switch.h"

#define LONG_OFF	0x01
#define SHORT_ON	0x02
#define LONG_ON		0x04
#define SHORT_OFF	0x08

#define TIME_SHROT	80	// Unit : millisecond
#define TIME_LONG	300 // Unit : millisecond

typedef struct {
	GPIO_TypeDef *port;	// Port of switch
	uint32_t pinMask;	// Pin of switch
	uint32_t timer;
	uint32_t prevTick;
	uint8_t state;
} ButtonState_t;

static void Custom_Switch_Init_ButtonState(ButtonState_t *State,
		GPIO_TypeDef *GPIOx, uint32_t PinMask) {
	State->port = GPIOx;
	State->pinMask = PinMask;
	State->timer = 0;
	State->prevTick = Custom_Delay_Get_SysTick();
	State->state = LONG_OFF;
}

static uint8_t Custom_Switch_State_Machine(ButtonState_t *State) {
	/**
	 * 이 함수는 State 구조체의 값을 기반으로 아래 설명된 바와 같이 state machine을 구현한다.
	 * 원래는 1ms마다 호출됨을 전제로 하였으나 OLED 화면 출력과 같은 시간이 많이 소요되는 함로로 인해 부하가 걸리면
	 * 1ms 시간이 부족하여 스위치가 동작이 제대로 수행되지 않는 문제점이 있었다.
	 * 함수가 호출된 시간을 측정하여 타이머 값에 시간을 뺀 뒤 기준 시간이 지났음을 감지하여 버튼 입력값을 출력하는 방식으로 변경하였다.
	 */
	bool currentPushed = !(LL_GPIO_ReadInputPort(State->port) & State->pinMask);
	bool pushEvent = false;

	uint32_t currTick = Custom_Delay_Get_SysTick();

	switch (State->state) {

		case LONG_OFF:
			if (currentPushed) {
				State->state = SHORT_ON;
				State->timer = TIME_SHROT;
			}
			break;

		case SHORT_ON:
			if (State->timer <= currTick - State->prevTick) {
				pushEvent = true;
				State->state = LONG_ON;
				State->timer = TIME_LONG;
			}
			State->timer -= currTick - State->prevTick;
			break;

		case LONG_ON:
			if (!currentPushed) {
				State->state = SHORT_OFF;
				State->timer = TIME_SHROT;
				break;
			}
			if (State->timer <= currTick - State->prevTick) {
				pushEvent = true;
				State->timer = TIME_LONG;
			}
			State->timer -= currTick - State->prevTick;
			break;

		case SHORT_OFF:
			if (State->timer <= currTick - State->prevTick) {
				State->state = LONG_OFF;
			}
			State->timer -= currTick - State->prevTick;
			break;
	}

	// 다음 호출 시 이전에 함수가 호출된 시간을 참조해야 하므로 prevTick값에 저장한다.
	State->prevTick = currTick;

	return pushEvent;
}

uint8_t Custom_Switch_Read(void) {
	/**
	 * 스위치가 눌리거나 떼질 때, 그 값이 한 번에 깔끔하게 변하지 않고 아주 짧은 시간동안 On, Off를 수십~수백번 반복한다.
	 * 이를 Bouncing 혹은 Chattering이라 한다. Bouncing의 특징은 일반적인 노이즈와 다르게
	 * 버튼을 누르는 동안 혹은 떼는 동안의 아주 짧은 시간동안만 발생하고, 버튼이 제대로 눌려있거나 떨어져있는 동안은 발생하지 않는다는 점이다.
	 * 이러한 특성을 고려하여 bouncing을 방지하기 위한 알고리즘을 설계하고자 한다.
	 *
	 * 우리 보드에는 버튼이 두 개 있으므로 이를 모두 고려해야 한다. 그러나 문제를 간단하게 하기 위해,
	 * 일단 버튼이 한 개인 경우를 먼저 생각한 후 버튼이 두 개인 경우를 고려하기로 한다.
	 * 1. 버튼이 한참동안 눌리지 않은 상태에서 갑자기 버튼이 눌린다면 버튼이 눌린 것으로 간주할 수 있다.
	 * 2. 버튼이 눌린 직후 버튼이 떼졌다고 하더라도, 이는 일시적인 바운싱에 불과하고 실제로 버튼이 떼진 것은 아니라고 간주할 수 있다.
	 * 3. 만약 1. 의 사건이 발생한 직후 버튼이 떼지고 다시 눌리지 않은 채로 충분한 시간이 지났다면 이는 버튼이 떼진 것으로 간주할 수 있다.
	 * 	    혹은, 버튼이 충분히 오랫동안 눌려있다가 갑자기 떼진 상태라면 이는 버튼을 떼는 동작으로 간주할 수 있다.
	 * 4. 버튼이 충분히 오랫동안 눌려있다가 버튼이 떼진 후, 얼마 지나지 않아 버튼이 다시 눌린다고 하더라도, 이는 일시적인 바운싱에 불과하고 다시 버튼이 눌린 것은 아니다.
	 *
	 * 즉, 이 알고리즘에 기반하여 생각할 때, 버튼의 상태를 다음과 같이 네 가지로 요약할 수 있다.
	 * 1. 오랫동안 눌리지 않은 상태	(Long-Off)
	 * 2. 눌러진 지 얼마 안 된 상태 	(Short-On)
	 * 3. 눌러진 지 오래 된 상태 	(Long-On)
	 * 4. 떼진 지 얼마 안 된 상태 	(Short-Off)
	 *
	 * Long-Off 상태에서 버튼이 눌릴 경우 Short-On상태로 진입한다.
	 * Short-On 상태에서는 바운싱을 방지하기 위해 모든 상태 변화를 무시한다.
	 * Short-On 상태가 끝나면 Long-On으로 진입하면서 버튼 눌림 이벤트를 발생시킨다.
	 * Long-On 상태에서는 버튼이 떼질 경우 Short-Off 상태로 진입한다.
	 * Short-Off 상태에서는 바운싱을 방지하기 위해 모든 상태 변화를 무시한다.
	 * Short-Off 상태가 끝나면 Long-Off로 진입한다. 떼는 이벤트도 발생시킬 수 있기는 하겠으나, 굳이 필요하지 않으므로 발생시키지 않기로 한다.
	 *
	 * 다음으로 버튼이 두 개인 경우를 고려하자.
	 * 버튼 두 개를 동시에 누를 경우, 버튼 두 개가 동시에 이벤트가 발생해야 한다.
	 * 그런데 위의 알고리즘을 따른다면  양쪽 버튼을 아주 짧은 시간 간격을 두고 누른다 하더라도 제대로 동시 입력으로 간주되지 않을 것이다.
	 * 물론 그 차이가 1ms이하라면 그런 문제가 발생하지 않겠지만, 이는 거의 불가능하다.
	 * 이를 해결하기 위해서는 한쪽 버튼이 Long-On으로 넘어갈 때 다른 쪽 버튼이 Short-On 상태에 있다면 강제로  Long-On상태로 넘겨버리면 된다.
	 *
	 * 마지막으로 버튼을 오랫동안 누르고 있을 때의 이벤트 발생 조건에 대해 고려하자.
	 * 오랫동안 버튼을 누르고 있을 때에는 충분히 긴 기간동안 눌렸다면 이벤트가 한 번 더 발생하면 된다.
	 * 그러므로 Long-On 상태에 있을 때 타이머를 하나 둔 후, 만약 그 타이머가 0 이하로 떨어질 경우 이벤트를 발생시키고 다시 타이머를 초기화하면 된다.
	 */

	/*
	 * Button의 상태를 초기화한다.
	 * structure를 바로 초기화하지 않고 함수를 사용하여 초기화하는 이유는 멤버 번수 순서 때문이다.
	 * 만약 structure의 정의 부분에서만 멤버 변수 순서를 바꾸고, 초기화하는 부분에서는 바꾸지 않는다면 예상과 다르게 작동할 것이다.
	 * 그러나 C언어 및 임베디드시스템에는 그런 오류를 잡는 기능이 없다. 그래서 이러한 함수를 사용하는 것이 바람직하다.
	 * 다만 매우 고속의 연산이 필요한 경우 if검사를 매번 실시하는 것은 바람직하지 않다. 그런 경우 초기화 루틴을 따로 함수로 빼는 것이 낫다.
	 */
	static bool isInitialized = false;
	static ButtonState_t sw1, sw2, sw3;
	if (!isInitialized) {
		isInitialized = true;
		Custom_Switch_Init_ButtonState(&sw1, SW1_PORT, SW1_PIN);
		Custom_Switch_Init_ButtonState(&sw2, SW2_PORT, SW2_PIN);
		Custom_Switch_Init_ButtonState(&sw3, SW3_PORT, SW3_PIN);
	}

	uint8_t sw1PushEvent = Custom_Switch_State_Machine(&sw1);
	uint8_t sw2PushEvent = Custom_Switch_State_Machine(&sw2);
	uint8_t sw3PushEvent = Custom_Switch_State_Machine(&sw3);

	/*
	 * 두 버튼이 동시에 눌리는 것을 처리하기 위해, 한 버튼의 푸시 이벤트가 발생했고 다른 버튼이 Short-On 상태에 있다면 두 버튼이 동시에 눌린 것으로 간주한다.
	 */
	if(sw1PushEvent) {
		if (sw2.state & SHORT_ON){
			sw2PushEvent = true;
			sw2.state = sw1.state;
			sw2.timer = sw1.timer;
		}
		if (sw3.state & SHORT_ON) {
			sw3PushEvent = true;
			sw3.state = sw1.state;
			sw3.timer = sw1.timer;
		}
	}
	else if(sw2PushEvent) {
		if (sw1.state & SHORT_ON){
			sw1PushEvent = true;
			sw1.state = sw2.state;
			sw1.timer = sw2.timer;
		}
		if (sw3.state & SHORT_ON) {
			sw3PushEvent = true;
			sw3.state = sw2.state;
			sw3.timer = sw2.timer;
		}
	}
	else if(sw3PushEvent) {
		if (sw1.state & SHORT_ON){
			sw1PushEvent = true;
			sw1.state = sw3.state;
			sw1.timer = sw3.timer;
		}
		if (sw2.state & SHORT_ON) {
			sw2PushEvent = true;
			sw2.state = sw3.state;
			sw2.timer = sw3.timer;
		}
	}


	uint8_t buttonPushEvent = 0;
	if (sw1PushEvent) buttonPushEvent |= CUSTOM_SW_1;
	if (sw2PushEvent) buttonPushEvent |= CUSTOM_SW_2;
	if (sw3PushEvent) buttonPushEvent |= CUSTOM_SW_3;

	return buttonPushEvent;
}
