/*
 * sensor.h
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include "main.h"
#include "drive_def_var.h"


#define INT_SWAP(a, b)				{ a ^= b; b ^= a; a ^= b; }
#define FLOAT_SWAP(a, b, c)			{ a = b; b = c; c = a; }


#define IR_SENSOR_LEN			16


#define WINDOW_SIZE_HALF		2


#define LINE_MARKER_SENSOR_LEN	5


#define	THRESHOLD_MAX 			250
#define	THRESHOLD_MIN			20
#define	THRESHOLD_CHANGE_VAL	5
#define	THRESHOLD_INIT			100


extern volatile uint8_t		sensorRawVals[IR_SENSOR_LEN];

extern volatile uint8_t		sensorNormVals[IR_SENSOR_LEN];
extern volatile uint8_t		normalizeCoef[IR_SENSOR_LEN];
extern volatile uint8_t		whiteMaxs[IR_SENSOR_LEN];
extern volatile uint8_t		blackMaxs[IR_SENSOR_LEN];

extern volatile uint16_t	state;
extern volatile uint8_t		threshold;

extern volatile int32_t		positionTable[IR_SENSOR_LEN];

extern volatile float		sensingVoltage;






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
		INT_SWAP(sensorMidian[0], sensorMidian[1]);
	}
	if (sensorMidian[1] > sensorMidian[2]) {
		INT_SWAP(sensorMidian[1], sensorMidian[2]);
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

/*
 * 	sensorNormVals[idx] = ( (255 * (sensorRawVals[idx] - blackMaxs[idx]) / normalizeCoef[idx]) \
 * 		& ( (sensorRawVals[idx] < blackMaxs[idx]) - 0x01 )  ) \
 * 		| ( (sensorRawVals[idx] < whiteMaxs[idx]) - 0x01 );
*/


	if (sensorRawVals[idx] < blackMaxs[idx])
		sensorNormVals[idx] = 0;
	else if (sensorRawVals[idx] > whiteMaxs[idx])
		sensorNormVals[idx] = 255;
	else
		sensorNormVals[idx] = (255 * (sensorRawVals[idx] - blackMaxs[idx]) / normalizeCoef[idx]);

}



// sensor state 계산
__STATIC_INLINE void	Make_Sensor_State(uint8_t idx) {

	state = ( state & ~(0x01 << idx) ) | ( (sensorNormVals[idx] > threshold ? 1 : 0) << idx );

//	if (sensorNormVals[idx] > threshold) {
//		state |= 0x01 << (IR_SENSOR_LEN - 1 - idx);
//	}
//	else {
//		state &= ~(0x01 << (IR_SENSOR_LEN - 1 - idx));
//	}
}



__STATIC_INLINE float	Make_Voltage_Raw_Val() {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
	return 3.3f * 21.f * (float)ADC_Read() / 4095.f;
}



__STATIC_INLINE void	Make_Battery_Voltage() {
	static uint8_t	sensingVoltageIdx = 0;
	static float	sensingVoltageMidian[3];


	switch(sensingVoltageIdx) {
		case 0:
		case 1:
		case 2:
			sensingVoltageMidian[sensingVoltageIdx] = Make_Voltage_Raw_Val();
			sensingVoltageIdx++;

			break;


		case 3:
			float tmp;

			if (sensingVoltageMidian[0] > sensingVoltageMidian[1]) {
				FLOAT_SWAP(tmp, sensingVoltageMidian[0], sensingVoltageMidian[1]);
			}
			if (sensingVoltageMidian[1] > sensingVoltageMidian[2]) {
				FLOAT_SWAP(tmp, sensingVoltageMidian[1], sensingVoltageMidian[2]);
			}
			if (sensingVoltageMidian[0] > sensingVoltageMidian[1]) {
				FLOAT_SWAP(tmp, sensingVoltageMidian[0], sensingVoltageMidian[1]);
			}

			sensingVoltage = sensingVoltageMidian[1];
			sensingVoltageIdx = 0;

			break;
	}
}



__STATIC_INLINE void	Position_Windowing() {

	int		window = (positionVal + 30000) / 4000;

	positionIdxMax = GET_MIN(window + WINDOW_SIZE_HALF, IR_SENSOR_LEN - 1);
	positionIdxMin = GET_MAX(window - WINDOW_SIZE_HALF + 1, 0);
}





__STATIC_INLINE void	Sum_Position_Val(uint8_t idx) {


	if (positionIdxMin <= idx && idx <= positionIdxMax) {

		positionSum += positionTable[idx] * sensorNormVals[idx];
		sensorNormValsSum += sensorNormVals[idx];
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
			Position_Windowing();
			Sum_Position_Val(tim5Idx);
			Sum_Position_Val(tim5Idx + 8);

			break;


		case 1:
		case 2:
			Sum_Position_Val(tim5Idx);
			Sum_Position_Val(tim5Idx + 8);

			break;


		case 3:
		case 4:
		case 5:
		case 6:
			Make_Battery_Voltage();
			Sum_Position_Val(tim5Idx);
			Sum_Position_Val(tim5Idx + 8);

			break;


		case 7:
			Sum_Position_Val(tim5Idx);
			Sum_Position_Val(tim5Idx + 8);
			Make_Position_Val();

			break;


	}

	// 인덱스 증가
	tim5Idx = (tim5Idx + 1) & 0x07;
}
//
//	0 0 0 0 // 0 0 0 0 // 0 0 0 0 // 0 0 0 0

#endif /* INC_SENSOR_H_ */
