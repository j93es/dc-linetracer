/*
 * sensor.h
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include "main.h"


#define SWAP(a, b)				{ a ^= b; b ^= a; a ^= b; }


#define	THRESHOLD_MAX 			250
#define	THRESHOLD_MIN			20
#define	THRESHOLD_CHANGE_VAL	5
#define	THRESHOLD_INIT			100



extern volatile uint8_t		sensorRawVals[16];
extern volatile uint8_t		midian[3];

extern volatile uint8_t		sensorNormVals[16];
extern volatile uint8_t		normalizeCoef[16];
extern volatile uint8_t		whiteMaxs[16];
extern volatile uint8_t		blackMaxs[16];

extern volatile uint16_t	state;
extern volatile uint8_t		threshold;






void	Sensor_Start();
void	Sensor_Stop();
void	Sensor_Calibration();



__STATIC_INLINE uint16_t	Sensor_ADC_Read() {
	uint16_t adcValue;
	__disable_irq();
	LL_ADC_ClearFlag_EOCS(ADC1);
	LL_ADC_REG_StartConversionSWStart(ADC1);
	while (!LL_ADC_IsActiveFlag_EOCS(ADC1));
	adcValue = LL_ADC_REG_ReadConversionData12(ADC1);
	LL_ADC_ClearFlag_EOCS(ADC1);
	__enable_irq();
	return adcValue;
}


__STATIC_INLINE void	Make_Sensor_Raw_Vals(uint8_t idx) {

	// 다음 IR LED 켜기
	GPIOC->ODR = (GPIOC->ODR & ~0x07) | idx | 0x08;
	if (idx < 8) {
		LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	} else {
		LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	}
	midian[0] = Sensor_ADC_Read() >> 4;
	midian[1] = Sensor_ADC_Read() >> 4;
	midian[2] = Sensor_ADC_Read() >> 4;

	// 선택한 IR LED 끄기
	GPIOC->ODR &= ~0x08;

	if (midian[0] > midian[1]) {
		SWAP(midian[0], midian[1]);
	}
	if (midian[1] > midian[2]) {
		SWAP(midian[1], midian[2]);
	}
	sensorRawVals[idx] = midian[1];
}


// normalized value 계산
__STATIC_INLINE void	Make_Sensor_Norm_Vals(uint8_t idx) {

	if (sensorRawVals[idx] < blackMaxs[idx])
		sensorNormVals[idx] = 0;
	else if (sensorRawVals[idx] > whiteMaxs[idx])
		sensorNormVals[idx] = 255;
	else
		sensorNormVals[idx] = (255 * (sensorRawVals[idx] - blackMaxs[idx]) / normalizeCoef[idx]);

}



// sensor state 계산
__STATIC_INLINE void	Make_Sensor_State(uint8_t idx) {

	if (sensorNormVals[idx] > threshold) {
		state |= 0x01 << (15 - idx);
	}
	else {
		state &= ~(0x01 << (15 - idx));
	}
}



__STATIC_INLINE void	Sensor_TIM5_IRQ() {
	static uint8_t	idx = 0;

	Make_Sensor_Raw_Vals(idx);
	Make_Sensor_Raw_Vals(idx + 8);

	Make_Sensor_Norm_Vals(idx);
	Make_Sensor_Norm_Vals(idx + 8);

	Make_Sensor_State(idx);
	Make_Sensor_State(idx + 8);

	// 인덱스 증가
	idx = (idx + 1) & 0x07;
}


#endif /* INC_SENSOR_H_ */
