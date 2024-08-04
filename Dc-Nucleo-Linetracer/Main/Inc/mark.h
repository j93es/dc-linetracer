/*
 * drive_state_machine.h
 */

#ifndef INC_MARK_H_
#define INC_MARK_H_


#include <config.h>
#include "sensor.h"
#include "main.h"
#include "math.h"





__STATIC_INLINE void	Mark_Masking(int32_t curIrSensorMid) {

	if (curIrSensorMid > IR_SENSOR_MID) {

		int8_t moveLen = curIrSensorMid - IR_SENSOR_MID;

		lineMasking = LINE_MASKING_INIT >> moveLen;
		leftMarkMasking = LEFT_MARK_MASKING_INIT >> moveLen;
		rightMarkMasking = RIGHT_MARK_MASKING_INIT >> moveLen;
		bothMarkMasking = leftMarkMasking | rightMarkMasking;
	} else {

		int8_t moveLen = IR_SENSOR_MID - curIrSensorMid;

		lineMasking = LINE_MASKING_INIT << moveLen;
		leftMarkMasking = LEFT_MARK_MASKING_INIT << moveLen;
		rightMarkMasking = RIGHT_MARK_MASKING_INIT << moveLen;
		bothMarkMasking = leftMarkMasking | rightMarkMasking;
	}

	markAreaMasking = ~(lineMasking << 1 | lineMasking >> 1);
}



__STATIC_INLINE void	Mark_Accumming(int32_t curIrSensorMid) {

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
	if (curIrSensorMid < 11) {

		irSensorStateSum |= (irSensorState & lineMasking) >> (11 - curIrSensorMid);
	} else {

		irSensorStateSum |= (irSensorState & lineMasking) << (curIrSensorMid - 11);
	}



	if ( __builtin_popcount(irSensorState & leftMarkMasking) != 0) {

		irSensorStateSum |= 0x80;
	}

	if ( __builtin_popcount(irSensorState & rightMarkMasking) != 0) {

		irSensorStateSum |= 0x01;
	}

}


__STATIC_INLINE void	Mark_Accumming_Reset() {

	irSensorStateSum = 0x00;
}








__STATIC_INLINE uint8_t	Is_Line_Out() {

	if (irSensorState == 0x00) {
		return CUSTOM_TRUE;
	}

	return CUSTOM_FALSE;
}




__STATIC_INLINE uint8_t	Is_Passed_Marker() {

	if ( __builtin_popcount(irSensorState & markAreaMasking) == 0 ) {
		return CUSTOM_TRUE;
	}

	return CUSTOM_FALSE;
}









// end line, right mark, left mark, straight를 판별하고 정해진 동작을 실행하는 함수
__STATIC_INLINE void	Mark_Decision() {

	// cross
	if (irSensorStateSum == 0xff) {

		markState = MARK_CROSS;
	}


	// end mark
	else if ((irSensorStateSum & 0x81) == 0x81) {

		markState = MARK_END;
	}


	// left mark
	else if ((irSensorStateSum & 0x80) == 0x80) {


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
	else if ((irSensorStateSum & 0x01) == 0x01) {

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








__STATIC_INLINE void	Mark() {

	static uint32_t	lineOutStartTime = 0;

	int32_t	curIrSensorMid = (positionVal + 30000) / 4000;

	Mark_Masking(curIrSensorMid);


	switch (markStateMachine) {


		case MARK_STATE_MACHINE_IDLE :

				// 라인 센서 4개 이상 인식
				if (__builtin_popcount(irSensorState & lineMasking) >= 4) {

					Mark_Accumming_Reset();
					Mark_Accumming(curIrSensorMid);
					markStateMachine = MARK_STATE_MACHINE_CROSS;
				}

				// 라인 센서 4개 이하 and 마크 센서 1개 이상
				else if (__builtin_popcount(irSensorState & bothMarkMasking) >= 1) {

					Mark_Accumming_Reset();
					Mark_Accumming(curIrSensorMid);
					markStateMachine = MARK_STATE_MACHINE_MARKER;
				}

				// 라인아웃되거나 잠깐 떳을 때
				else if (Is_Line_Out()) {

					lineOutStartTime = uwTick;

					markStateMachine = MARK_STATE_MACHINE_LINE_OUT;
				}

				break;





		case MARK_STATE_MACHINE_CROSS:

				// accum
				Mark_Accumming(curIrSensorMid);

				// 모든 센서를 읽었고 마크 센서가 선을 지나쳤을 때 IDLE
				if ( (irSensorStateSum == 0xff && Is_Passed_Marker()) \
					|| Is_Line_Out() ) {

					markStateMachine = MARK_STATE_MACHINE_DECISION;
				}

				break;





		case MARK_STATE_MACHINE_MARKER :

				// accum
				Mark_Accumming(curIrSensorMid);

				// 마커 센서가 0개 일 때
				if (Is_Passed_Marker() || Is_Line_Out()) {

					markStateMachine = MARK_STATE_MACHINE_DECISION;
				}

				break;





		case MARK_STATE_MACHINE_DECISION :

				Mark_Decision();

				markStateMachine = MARK_STATE_MACHINE_IDLE;

				break;





		case MARK_STATE_MACHINE_LINE_OUT :


				if (!Is_Line_Out()) {

					markStateMachine = MARK_STATE_MACHINE_IDLE;
				}

				// state == 0x00인 상태가 t(ms) 지속되었을 때
				else if (uwTick > lineOutStartTime + LINE_OUT_DELAY_MS) {

					markState = MARK_LINE_OUT;
				}

				break ;

	}
}



#endif
