/*
 * drive_state_machine.h
 */

#ifndef INC_MARK_SAMPLING_H_
#define INC_MARK_SAMPLING_H_


#include <config.h>
#include "sensor.h"
#include "positioning.h"
#include "main.h"
//#include "mark2.h"




__STATIC_INLINE uint8_t	Sensor_State_8(int8_t curIrSensorMid) {

	// 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0
	// ~
	// 7 => 4
	// 8 => 3
	// 9 => 2
	// 10 => 1
	// 11 => 0
	// 12 <= 1
	// 13 <= 2
	// 14 <= 3
	// 15 <= 4

	uint8_t sensor8Result = 0x00;

	if (curIrSensorMid < 11) {

		sensor8Result |= (irSensorState & lineMasking) >> (11 - curIrSensorMid);
	} else {

		sensor8Result |= (irSensorState & lineMasking) << (curIrSensorMid - 11);
	}


	if ( __builtin_popcount(irSensorState & leftMarkMasking) != 0) {

		sensor8Result |= 0x80;
	}

	if ( __builtin_popcount(irSensorState & rightMarkMasking) != 0) {

		sensor8Result |= 0x01;
	}

	return sensor8Result;

}





__STATIC_INLINE void	Mark_Sampling_Reset() {

	targetMarkSamplingTick = 0;
	curMarkSamplingTick = 0;


	for (uint8_t i = 0; i < MARK_SAMPLING_MAX_LEN; i++) {
		markSampling[i] = 0x00;
	}
}


__STATIC_INLINE uint8_t	Mark_Sampling(int8_t curIrSensorMid) {

	static uint8_t curMarkSamplingIdx = 0;

	// markSamplingCurTick은 timer9에서 증가
	if (targetMarkSamplingTick > curMarkSamplingTick) {

		return 0;
	}

	markSampling[curMarkSamplingIdx] = Sensor_State_8(curIrSensorMid);

	targetMarkSamplingTick +=  MARK_SAMPLING_TICK;

	if (curMarkSamplingIdx < MARK_SAMPLING_MAX_LEN) {
		curMarkSamplingIdx += 1;
	} else {
		curMarkSamplingIdx = 0;
	}

	return 1;
}






#endif
