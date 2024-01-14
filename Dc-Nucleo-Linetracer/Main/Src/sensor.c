/*
 * sensor.c
 */

#include "header_init.h"



volatile uint8_t	sensorRawVals[16];

volatile uint8_t	sensorNormVals[16];
volatile uint8_t	normalizeCoef[16];
volatile uint8_t	whiteMaxs[16];
volatile uint8_t	blackMaxs[16];

volatile uint8_t	state = 0x00;
volatile uint8_t	threshold = THRESHOLD_INIT;

volatile int32_t	positionTable[16] = { -14000, -12000, -10000, -8000, -6000, -4000, -2000, 0, 0,\
		2000, 4000, 6000, 8000, 10000, 12000, 14000 };

volatile float		voltage;






void Sensor_Start() {

	LL_ADC_Enable(ADC1);
	Custom_Delay_ms(10); // ADC를 켜고 난 후, ADC 변환을 하기 전 내부 아날로그 안정화 작업을 위해 딜레이를 준다.

	LL_TIM_EnableCounter(TIM5); // TIM5의 타이머 카운터가 증가하도록 설정한다.
	LL_TIM_EnableIT_UPDATE(TIM5); // TIM5의 인터럽트가 동작하도록 설정한다. 인터럽트가 발생하면 Core/Src/stm32f4xx_it.c 파일 내부에 있는 인터럽트 핸들러 함수가 호출된다.
	Custom_Delay_ms(10); // 센서 raw 값이 생성될 때까지 기다림
}




void Sensor_Stop() {
	LL_ADC_Disable(ADC1);
	LL_TIM_DisableCounter(TIM5);
	LL_TIM_DisableIT_UPDATE(TIM5);
}





void Sensor_Calibration() {
	uint8_t	tmp = 0;

	for (uint8_t i = 0; i < 8; i++) {
		whiteMaxs[i] = 0;
		blackMaxs[i] = 0;
	}

	Sensor_Start();

	// Get blackMax
	Custom_OLED_Clear();
	while (CUSTOM_SW_3 != Custom_Switch_Read()) {
		Custom_OLED_Printf("/0Black Max");

		for (uint8_t i = 0; i < 16; i++) {
			if (blackMaxs[i] < (tmp = sensorRawVals[i])) {
				blackMaxs[i] = tmp;
			}
		}
		Custom_OLED_Printf("/1%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			blackMaxs[0], blackMaxs[1], blackMaxs[2], blackMaxs[3], \
			blackMaxs[4], blackMaxs[5], blackMaxs[6], blackMaxs[7]);

		Custom_OLED_Printf("/3%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			blackMaxs[8], blackMaxs[9], blackMaxs[10], blackMaxs[11], \
			blackMaxs[12], blackMaxs[13], blackMaxs[14], blackMaxs[15]);
	}

	// Get whiteMax
	Custom_OLED_Clear();
	while (CUSTOM_SW_3 != Custom_Switch_Read()) {
		Custom_OLED_Printf("/0White Max");

		for (uint8_t i = 0; i < 16; i++) {
			if (whiteMaxs[i] < (tmp = sensorRawVals[i])) {
				whiteMaxs[i] = tmp;
			}
		}
		Custom_OLED_Printf("/1%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			whiteMaxs[0], whiteMaxs[1], whiteMaxs[2], whiteMaxs[3], \
			whiteMaxs[4], whiteMaxs[5], whiteMaxs[6], whiteMaxs[7]);

		Custom_OLED_Printf("/3%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			whiteMaxs[8], whiteMaxs[9], whiteMaxs[10], whiteMaxs[11], \
			whiteMaxs[12], whiteMaxs[13], whiteMaxs[14], whiteMaxs[15]);
	}

	// Calculate ADC coefficients
	for (uint8_t i = 0; i < 16; i++) {
		normalizeCoef[i] = whiteMaxs[i] - blackMaxs[i];
	}

	Custom_OLED_Clear();
	Sensor_Stop();
}
