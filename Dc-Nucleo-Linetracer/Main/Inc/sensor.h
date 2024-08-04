/*
 * sensor.h
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include <config.h>
#include "main.h"

#include "core_cm4.h"



extern volatile uint8_t		sensorRawVals[IR_SENSOR_LEN];

extern volatile uint8_t		sensorNormVals[IR_SENSOR_LEN];
extern volatile uint8_t		normalizeCoef[IR_SENSOR_LEN];
extern volatile uint8_t		whiteMaxs[IR_SENSOR_LEN];
extern volatile uint8_t		blackMaxs[IR_SENSOR_LEN];

extern volatile uint16_t	irSensorState;
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





__STATIC_INLINE void	Make_Sensor_Raw_Vals(uint8_t idx) {

	// IR LED 켜기
	GPIOC->ODR = (GPIOC->ODR & ~0x07) | idx | 0x08;

	uint16_t sensorMidianLeft[3];
	uint16_t sensorMidianRight[3];



	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	sensorMidianLeft[0] = ADC_Read();

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	sensorMidianRight[0] = ADC_Read();



	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	sensorMidianLeft[0] = ADC_Read();

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	sensorMidianRight[0] = ADC_Read();



	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	sensorMidianLeft[1] = ADC_Read();

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	sensorMidianRight[1] = ADC_Read();



	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
	sensorMidianLeft[2] = ADC_Read();

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	sensorMidianRight[2] = ADC_Read();



	if (sensorMidianLeft[0] > sensorMidianLeft[1]) {
		INT_SWAP(sensorMidianLeft[0], sensorMidianLeft[1]);
	}
	if (sensorMidianLeft[1] > sensorMidianLeft[2]) {
		INT_SWAP(sensorMidianLeft[1], sensorMidianLeft[2]);
	}
	if (sensorMidianLeft[0] > sensorMidianLeft[1]) {
		INT_SWAP(sensorMidianLeft[0], sensorMidianLeft[1]);
	}



	if (sensorMidianRight[0] > sensorMidianRight[1]) {
		INT_SWAP(sensorMidianRight[0], sensorMidianRight[1]);
	}
	if (sensorMidianRight[1] > sensorMidianRight[2]) {
		INT_SWAP(sensorMidianRight[1], sensorMidianRight[2]);
	}
	if (sensorMidianRight[0] > sensorMidianRight[1]) {
		INT_SWAP(sensorMidianRight[0], sensorMidianRight[1]);
	}



	sensorRawVals[idx] = sensorMidianLeft[1] >> 4;
	sensorRawVals[idx + 8] = sensorMidianRight[1] >> 4;

	// 선택한 IR LED 끄기
	GPIOC->ODR &= ~0x08;
}




// normalized value 계산
__STATIC_INLINE void	Make_Sensor_Norm_Vals(uint8_t idx) {


 	sensorNormVals[idx] = ( (255 * (sensorRawVals[idx] - blackMaxs[idx]) / normalizeCoef[idx]) \
 		& ( (sensorRawVals[idx] < blackMaxs[idx] ? 1 : 0) - 0x01 )  ) \
 		| ( (sensorRawVals[idx] < whiteMaxs[idx] ? 1 : 0) - 0x01 );



//	if (sensorRawVals[idx] < blackMaxs[idx]) {
//		sensorNormVals[idx] = 0;
//	}
//	else if (sensorRawVals[idx] > whiteMaxs[idx]) {
//		sensorNormVals[idx] = 255;
//	}
//	else {
//		sensorNormVals[idx] = (255 * (sensorRawVals[idx] - blackMaxs[idx]) / normalizeCoef[idx]);
//	}
}



// sensor state 계산
__STATIC_INLINE void	Make_Sensor_State(uint8_t idx) {

	uint8_t stateMaskingIdx = IR_SENSOR_LEN - 1 - idx;

	irSensorState = ( irSensorState & ~(0x01 << stateMaskingIdx) ) | ( (sensorNormVals[idx] > threshold ? 1 : 0) << stateMaskingIdx );

//	if (sensorNormVals[idx] > threshold) {
//		irSensorState |= 0x01 << stateMaskingIdx;
//	}
//	else {
//		irSensorState &= ~(0x01 << stateMaskingIdx);
//	}
}



__STATIC_INLINE float	Make_Voltage_Raw_Val() {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
	return 3.3f * 21.f * (float)ADC_Read() / 4096.f;
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

			if (sensingVoltageMidian[0] > sensingVoltageMidian[1]) {
				FLOAT_SWAP(sensingVoltageMidian[0], sensingVoltageMidian[1]);
			}
			if (sensingVoltageMidian[1] > sensingVoltageMidian[2]) {
				FLOAT_SWAP(sensingVoltageMidian[1], sensingVoltageMidian[2]);
			}
			if (sensingVoltageMidian[0] > sensingVoltageMidian[1]) {
				FLOAT_SWAP(sensingVoltageMidian[0], sensingVoltageMidian[1]);
			}

			if (sensingVoltageMidian[1] == 0) {
				sensingVoltageMidian[1] = 0.001f;
			}
			sensingVoltage = sensingVoltageMidian[1];
			sensingVoltageIdx = 0;

			break;
	}
}





__STATIC_INLINE void	Sensor_TIM5_IRQ() {
	DWT->CYCCNT = 0;

	static uint8_t	tim5Idx = 0;

	Make_Sensor_Raw_Vals(tim5Idx);

	Make_Sensor_Norm_Vals(tim5Idx);
	Make_Sensor_Norm_Vals(tim5Idx + 8);

	Make_Sensor_State(tim5Idx);
	Make_Sensor_State(tim5Idx + 8);

	if (tim5Idx & 0x01) {
		Make_Battery_Voltage();
	}


//	다음의 순서로 센서를 읽음 { 0, 2, 4, 6, 1, 3, 5, 7 };
//	인덱스 증가
	tim5Idx += 2;

	if (tim5Idx == 9) {
		tim5Idx = 0;
	}
	else if (tim5Idx == 8) {
		tim5Idx = 1;
	}
}

#endif /* INC_SENSOR_H_ */
