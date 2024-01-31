/*
 * drive_state_machine.h
 */

#ifndef INC_DRIVE_STATE_MACHINE_H_
#define INC_DRIVE_STATE_MACHINE_H_


#include "drive_def_var.h"
#include "sensor.h"
#include "main.h"






__STATIC_INLINE void	Mark_Masking() {

	lineMasking = LINE_MASKING_TOOL >> (positionIdxMin - 1);
	rightMarkMasking = MARK_MASKING_TOOL >> GET_MIN(positionIdxMax + 3, IR_SENSOR_LEN - 2);
	leftMarkMasking = MARK_MASKING_TOOL >> GET_MAX(positionIdxMin - 4, 0);
	bothMarkMasking = rightMarkMasking | leftMarkMasking;
}




// end line, right mark, left mark, straight를 판별하고 정해진 동작을 실행하는 함수
__STATIC_INLINE void	Decision(uint16_t sensorStateSum) {

	// cross
	if (sensorStateSum == ALL_MARK_MASKING) {

		markState = MARK_CROSS;
	}


	// end mark
	// if ( ((sensorStateSum >> 0) & 0x01) && ((sensorStateSum >> 7) & 0x01) )
	else if ( (sensorStateSum & bothMarkMasking) == bothMarkMasking ) {

		markState = MARK_END;
	}


	// left mark
	else if ( (sensorStateSum & leftMarkMasking) == leftMarkMasking ) {


		// 이전 마크가 왼쪽 곡선 마크였다면 곡선주행 종료
		if (markState == MARK_CURVE_L) {
			markState = MARK_STRAIGHT;
		}

		// 곡선주행 진입
		else {
			markState = MARK_CURVE_L;
		}
	}


	// right mark
	else if ( (sensorStateSum & rightMarkMasking) == rightMarkMasking ) {

		// 이전 마크가 오른쪽 곡선 마크였다면 곡선주행 종료
		if (markState == MARK_CURVE_R) {
			markState = MARK_STRAIGHT;
		}

		// 곡선주행 진입
		else {
			markState = MARK_CURVE_R;
		}
	}
}





__STATIC_INLINE void	Drive_State_Machine() {

	static uint32_t	lineOutStartTime;


	switch (driveState) {


		case DRIVE_STATE_IDLE :

				Mark_Masking();

				// 라인 센서 4개 이상 인식
				if (__builtin_popcount(state & lineMasking) >= 4) {

					sensorStateSum = 0x00;

					driveState = DRIVE_STATE_CROSS;
				}

				// 라인 센서 4개 이하 and 마크 센서 1개 이상
				else if (__builtin_popcount(state & bothMarkMasking) != 0) {

					sensorStateSum = 0x00;

					driveState = DRIVE_STATE_MARKER;
				}

				// 라인아웃되거나 잠깐 떳을 때
				else if (state == 0x00) {

					lineOutStartTime = uwTick;

					driveState = DRIVE_DECISION_LINE_OUT;
				}

				break;





		case DRIVE_STATE_CROSS:

				// accum
				sensorStateSum |= state;

				// 모든 센서를 읽었고 마크 센서가 선을 지나쳤을 때 IDLE
				if (sensorStateSum == ALL_MARK_MASKING && __builtin_popcount(state & bothMarkMasking) == 0) {

					driveState = DRIVE_STATE_DECISION;
				}

				break;





		case DRIVE_STATE_MARKER :

				// accum
				sensorStateSum |= state;

				// 마커 센서가 0개 일 때
				if (__builtin_popcount(state & bothMarkMasking) == 0) {

					driveState = DRIVE_STATE_DECISION;
				}

				break;





		case DRIVE_STATE_DECISION :

				Decision(sensorStateSum);

				driveState = DRIVE_STATE_IDLE;

				break;





		case DRIVE_DECISION_LINE_OUT :

				if (state != 0x00) {

					driveState = DRIVE_STATE_IDLE;
				}

				// state == 0x00인 상태가 t(ms) 지속되었을 때
				else if (uwTick > lineOutStartTime + LINE_OUT_DELAY_MS) {

					markState = MARK_LINE_OUT;
				}

				break ;

	}
}



#endif
