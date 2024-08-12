/*
 * drive_state_machine.h
 */

#ifndef INC_MARK_H_
#define INC_MARK_H_


#include <config.h>
#include "sensor.h"
#include "main.h"
#include "math.h"




__STATIC_INLINE  void Mark_Masking_Init() {

	markMasking.line_mask[0] =	0b1111000000000000;
	markMasking.left_mask[0] =	0b0000000000000000;
	markMasking.right_mask[0] =	0b0000111100000000;

	markMasking.line_mask[1] =	0b1111100000000000;
	markMasking.left_mask[1] =	0b0000000000000000;
	markMasking.right_mask[1] =	0b0000011110000000;

	markMasking.line_mask[2] =	0b1111110000000000;
	markMasking.left_mask[2] =	0b0000000000000000;
	markMasking.right_mask[2] =	0b0000001111000000;

	markMasking.line_mask[3] =	0b0111111000000000;
	markMasking.left_mask[3] =	0b1000000000000000;
	markMasking.right_mask[3] =	0b0000000111100000;

	markMasking.line_mask[4] =	0b0011111100000000;
	markMasking.left_mask[4] =	0b1100000000000000;
	markMasking.right_mask[4] =	0b0000000011110000;

	markMasking.line_mask[5] =	0b0001111110000000;
	markMasking.left_mask[5] =	0b1110000000000000;
	markMasking.right_mask[5] =	0b0000000001111000;

	markMasking.line_mask[6] =	0b0000111111000000;
	markMasking.left_mask[6] =	0b0111000000000000;
	markMasking.right_mask[6] =	0b0000000000111100;

	markMasking.line_mask[7] =	0b0000011111100000;
	markMasking.left_mask[7] =	0b0011100000000000;
	markMasking.right_mask[7] =	0b0000000000011100;

	markMasking.line_mask[8] =	0b0000001111110000;
	markMasking.left_mask[8] =	0b0011110000000000;
	markMasking.right_mask[8] =	0b0000000000001110;

	markMasking.line_mask[9] =	0b0000000111111000;
	markMasking.left_mask[9] =	0b0001111000000000;
	markMasking.right_mask[9] =	0b0000000000000111;

	markMasking.line_mask[10] =	0b0000000011111100;
	markMasking.left_mask[10] =	0b0000111100000000;
	markMasking.right_mask[10] =0b0000000000000011;

	markMasking.line_mask[11] =	0b0000000001111110;
	markMasking.left_mask[11] =	0b0000011110000000;
	markMasking.right_mask[11] =0b0000000000000001;

	markMasking.line_mask[12] =	0b0000000000111111;
	markMasking.left_mask[12] =	0b0000001111000000;
	markMasking.right_mask[12] =0b0000000000000000;

	markMasking.line_mask[13] =	0b0000000000011111;
	markMasking.left_mask[13] =	0b0000000111100000;
	markMasking.right_mask[13] =0b0000000000000000;

	markMasking.line_mask[14] =	0b0000000000001111;
	markMasking.left_mask[14] =	0b0000000011110000;
	markMasking.right_mask[14] =0b0000000000000000;

	markMasking.line_mask[15] = 0b0000000000000111;
	markMasking.left_mask[15] =	0b0000000001111000;
	markMasking.right_mask[15] =0b0000000000000000;
}



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

//	lineMasking = markMasking.line_mask[curIrSensorMid];
//	leftMarkMasking = markMasking.left_mask[curIrSensorMid];
//	rightMarkMasking = markMasking.right_mask[curIrSensorMid];
//	bothMarkMasking = leftMarkMasking | rightMarkMasking;

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
				if (__builtin_popcount(irSensorState & lineMasking) >= 6) {

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
