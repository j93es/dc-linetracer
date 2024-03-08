/*
 * drive_positioning.h
 */

#ifndef INC_DRIVE_POSITIONING_H_
#define INC_DRIVE_POSITIONING_H_

#include "drive_def_var.h"
#include "main.h"
#include "motor.h"



__STATIC_INLINE void	Position_Windowing() {

	int32_t	window = (positionVal + 30000) / 4000;

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


__STATIC_INLINE void	Positioning(uint8_t *idx) {

	switch(*idx) {
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
				Sum_Position_Val(*idx);
				Sum_Position_Val(*idx + 8);
				*idx += 1;

				break;


			case 8:
				Make_Position_Val();
				Position_Windowing();

				*idx = 0;
				break;
	}
}

#endif
