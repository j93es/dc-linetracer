/*
 * drive_positioning.h
 */

#ifndef INC_POSITIONING_H_
#define INC_POSITIONING_H_

#include <config.h>
#include "main.h"
#include "motor.h"



__STATIC_INLINE void	Positioning() {


	int32_t positionSum = 0;
	int32_t sensorNormValsSum = 0;

	for (uint8_t i = positionIdxMin; i < positionIdxMax + 1; i++) {

		positionSum += positionTable[i] * sensorNormVals[i];
		sensorNormValsSum += sensorNormVals[i];
	}

	positionVal = positionSum / GET_MAX(sensorNormValsSum, 1);


	int8_t	curIrSensorMid = (positionVal + 30000) / 4000;

	curPositionIrSensorMid = curIrSensorMid;

	positionIdxMax = GET_MIN(curIrSensorMid + WINDOW_SIZE_HALF, IR_SENSOR_LEN - 1);
	positionIdxMin = GET_MAX(curIrSensorMid - WINDOW_SIZE_HALF + 1, 0);


//	// this!!!
//	if (curInlineVal != 0) {
//		positionIdxMax = GET_MAX(positionIdxMax, IR_SENSOR_LEN - 1 - positionIdxMin);
//		positionIdxMin = GET_MIN(positionIdxMin, IR_SENSOR_LEN - 1 - positionIdxMax);
//	}
}

#endif
