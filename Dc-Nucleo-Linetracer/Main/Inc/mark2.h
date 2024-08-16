/*
 * drive_state_machine.h
 */

#ifndef INC_MARK2_H_
#define INC_MARK2_H_


#include <config.h>
#include "sensor.h"
#include "main.h"
#include "mark_sampling.h"



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



__STATIC_INLINE void	Mark_Masking(int8_t curIrSensorMid) {

	lineMasking = markMasking.line_mask[curIrSensorMid];
	leftMarkMasking = markMasking.left_mask[curIrSensorMid];
	rightMarkMasking = markMasking.right_mask[curIrSensorMid];
	bothMarkMasking = leftMarkMasking | rightMarkMasking;

	markAreaMasking = ~(lineMasking << 1 | lineMasking >> 1);
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



__STATIC_INLINE void	Mark_Accumming() {


	irSensorStateSum = 0x00;

	for (uint8_t i = 0; i < MARK_SAMPLING_MAX_LEN; i++) {
		irSensorStateSum |= markSampling[i];
	}

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

	static uint8_t samplingCnt = 0;

	int8_t	curIrSensorMid = curPositionIrSensorMid;

	Mark_Masking(curIrSensorMid);


	samplingCnt += Mark_Sampling(curIrSensorMid);


	switch (markStateMachine) {


		case MARK_STATE_MACHINE_IDLE :

				samplingCnt = 0;

				// 라인 센서 4개 이상 인식
				if (__builtin_popcount(irSensorState & lineMasking) == 6) {

					markStateMachine = MARK_STATE_MACHINE_CROSS;
				}

				// 라인 센서 4개 이하 and 마크 센서 1개 이상
				else if (__builtin_popcount(irSensorState & bothMarkMasking) >= 1) {

					markStateMachine = MARK_STATE_MACHINE_MARKER;
				}

				// 라인아웃되거나 잠깐 떳을 때
				else if (Is_Line_Out()) {

					lineOutStartTime = uwTick;

					markStateMachine = MARK_STATE_MACHINE_LINE_OUT;
				}

				break;





		case MARK_STATE_MACHINE_CROSS:

				Mark_Accumming();

				if ( ((samplingCnt > MARK_SAMPLING_MAX_LEN - 1) || (irSensorStateSum == 0xff)) && Is_Passed_Marker() ) {

					markStateMachine = MARK_STATE_MACHINE_DECISION;
				}

				break;





		case MARK_STATE_MACHINE_MARKER :

				Mark_Accumming();

				if ( (samplingCnt > MARK_SAMPLING_MAX_LEN - 1) && Is_Passed_Marker()) {

					markStateMachine = MARK_STATE_MACHINE_DECISION;
				}

				break;





		case MARK_STATE_MACHINE_DECISION :

				Mark_Decision();
				Mark_Sampling_Reset();
				samplingCnt = 0;

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
