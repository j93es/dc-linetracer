/*
 * sensor.h
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include "main.h"
#include "drive_def_var.h"


#define SWAP(a, b)				{ a ^= b; b ^= a; a ^= b; }


#define	THRESHOLD_MAX 			250
#define	THRESHOLD_MIN			20
#define	THRESHOLD_CHANGE_VAL	5
#define	THRESHOLD_INIT			100


extern volatile uint8_t		sensorRawVals[16];

extern volatile uint8_t		sensorNormVals[16];
extern volatile uint8_t		normalizeCoef[16];
extern volatile uint8_t		whiteMaxs[16];
extern volatile uint8_t		blackMaxs[16];

extern volatile uint8_t		state;
extern volatile uint8_t		threshold;

extern volatile int32_t		positionTable[16];

extern volatile float		voltage;






void	Sensor_Start();
void	Sensor_Stop();
void	Sensor_Calibration();



__STATIC_INLINE uint16_t	ADC_Read() {
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




__STATIC_INLINE uint8_t	Sensor_ADC_Midian_Filter() {
	uint16_t sensorMidian[3];

	sensorMidian[0] = ADC_Read();
	sensorMidian[1] = ADC_Read();
	sensorMidian[2] = ADC_Read();

	if (sensorMidian[0] > sensorMidian[1]) {
		SWAP(sensorMidian[0], sensorMidian[1]);
	}
	if (sensorMidian[1] > sensorMidian[2]) {
		SWAP(sensorMidian[1], sensorMidian[2]);
	}

	return sensorMidian[1] >> 4;
}






__STATIC_INLINE void	Make_Sensor_Raw_Vals(uint8_t idx) {

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	sensorRawVals[idx] = Sensor_ADC_Midian_Filter();

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	sensorRawVals[idx + 8] = Sensor_ADC_Midian_Filter();
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



__STATIC_INLINE void	Make_Battery_Voltage() {
	static uint8_t	voltageIdx = 0;
	static uint16_t	voltageMidian[3];


	switch(voltageIdx) {
		case 0:
			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
			voltageMidian[voltageIdx] = 3.3f / 4095.f *21.f / 1.0f * (float)ADC_Read();

			voltageIdx++;
			break;

		case 1:
			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
			voltageMidian[voltageIdx] = 3.3f / 4095.f *21.f / 1.0f * (float)ADC_Read();

			voltageIdx++;
			break;

		case 2:
			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
			voltageMidian[voltageIdx] = 3.3f / 4095.f *21.f / 1.0f * (float)ADC_Read();

			voltageIdx++;
			break;

		case 3:
			if (voltageMidian[0] > voltageMidian[1]) {
				SWAP(voltageMidian[0], voltageMidian[1]);
			}
			if (voltageMidian[1] > voltageMidian[2]) {
				SWAP(voltageMidian[1], voltageMidian[2]);
			}
			if (voltageMidian[0] > voltageMidian[1]) {
				SWAP(voltageMidian[0], voltageMidian[1]);
			}

			voltageIdx = 0;
			break;
	}
}







__STATIC_INLINE void	Sum_Position_Val(uint8_t idx) {


	if (positionIdxMin <= idx && idx <= positionIdxMax) {

		positionSum += positionTable[idx] * sensorNormVals[idx];
		sensorNormValsSum += sensorNormVals[idx];
	}

	if (positionIdxMin <= idx + 8 && idx + 8 <= positionIdxMax) {

		positionSum += positionTable[idx + 8] * sensorNormVals[idx + 8];
		sensorNormValsSum += sensorNormVals[idx + 8];
	}
}



__STATIC_INLINE void	Position_Windowing() {

	positionIdxMax = 10;
	positionIdxMin = 5;

	if (ABS(positionVal) > positionTable[6]) {

		// positionVal이 -2000보다 작을 때
		if (positionVal < 0) {
			positionIdxMax = 7;
			positionIdxMin = 2;
		}
		// positionVal이 2000보다 클 때
		else {
			positionIdxMax = 13;
			positionIdxMin = 8;
		}
	}

}





__STATIC_INLINE void	Make_Position_Val() {

		positionVal = positionSum / (sensorNormValsSum + 1);

		positionSum = 0;
		sensorNormValsSum = 0;
}



__STATIC_INLINE void	Sensor_TIM5_IRQ() {
	static uint8_t	tim5Idx = 0;

	// 다음 IR LED 켜기
	GPIOC->ODR = (GPIOC->ODR & ~0x07) | tim5Idx | 0x08;

	Make_Sensor_Raw_Vals(tim5Idx);

	// 선택한 IR LED 끄기
	GPIOC->ODR &= ~0x08;

	Make_Sensor_Norm_Vals(tim5Idx);
	Make_Sensor_Norm_Vals(tim5Idx + 8);

	Make_Sensor_State(tim5Idx);
	Make_Sensor_State(tim5Idx + 8);



	switch(tim5Idx) {
		case 0:
//			Position_Windowing();
//			Sum_Position_Val(tim5Idx);
			break;

		case 1:
			break;

		case 2:
//			Sum_Position_Val(tim5Idx);
			break;

		case 3:
//			Sum_Position_Val(tim5Idx);
			Make_Battery_Voltage();
			break;

		case 4:
//			Sum_Position_Val(tim5Idx);
			Make_Battery_Voltage();
			break;

		case 5:
//			Sum_Position_Val(tim5Idx);
			Make_Battery_Voltage();
			break;

		case 6:
//			Sum_Position_Val(tim5Idx);
			Make_Battery_Voltage();
			break;

		case 7:
//			Sum_Position_Val(tim5Idx);
//			Make_Position_Val();
			break;


	}

	// 인덱스 증가
	tim5Idx = (tim5Idx + 1) & 0x07;
}
//
//	0 0 0 0 // 0 0 0 0 // 0 0 0 0 // 0 0 0 0

#endif /* INC_SENSOR_H_ */
