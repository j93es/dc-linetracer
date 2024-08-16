
#ifndef INC_INLINE_DERIVE_H_
#define INC_INLINE_DERIVE_H_

#include <config.h>
#include "main.h"

__STATIC_INLINE void Inline_Drive() {

	if (!isInlineDriveEnabled) {
		return;
	}

	if (isLastStraight) {
		targetInlineVal = 0;
		return;
	}

	if (driveData[driveDataIdx].tickCnt_L > curTick_L + INLINE_POSITIONING_TICK
		&& driveData[driveDataIdx].tickCnt_R > curTick_R + INLINE_POSITIONING_TICK) {

		return;
	}

	if (driveData[driveDataIdx + 1].markState == MARK_CURVE_R) {

		targetInlineVal = -1 * ABS_INLINE_TARGET_POSITION;
		return;
	}

	if (driveData[driveDataIdx + 1].markState == MARK_CURVE_L) {

		targetInlineVal = ABS_INLINE_TARGET_POSITION;
		return;
	}

	if (driveData[driveDataIdx + 1].markState == MARK_STRAIGHT) {

		if (driveData[driveDataIdx].crossCnt != driveData[driveDataIdx + 1].crossCnt) {

			targetInlineVal = 0;
			return;
		}

		if (driveData[driveDataIdx].markState != driveData[driveDataIdx + 2].markState) {

			targetInlineVal = 0;
			return;
		}
	}
}


//
//__STATIC_INLINE void Prepare_Inline() {
//
//	if (!isInlineDriveEnabled) {
//		return;
//	}
//
//	if (isLastStraight) {
//		targetInlineVal = 0;
//		return;
//	}
//
//	if (driveData[driveDataIdx].tickCnt_L > curTick_L + INLINE_PREPARE_POSITIONING_TICK
//		&& driveData[driveDataIdx].tickCnt_R > curTick_R + INLINE_PREPARE_POSITIONING_TICK) {
//
//		return;
//	}
//
//	if (driveData[driveDataIdx].crossCnt != driveData[driveDataIdx + 1].crossCnt) {
//
//		return;
//	}
//
//	if (driveData[driveDataIdx + 1].markState == MARK_CURVE_R) {
//
//		targetInlineVal = -1 * ABS_INLINE_TARGET_POSITION;
//	}
//	else if (driveData[driveDataIdx + 1].markState == MARK_CURVE_L) {
//
//		targetInlineVal = ABS_INLINE_TARGET_POSITION;
//	}
//
//}
//
//__STATIC_INLINE void Restore_Inline() {
//	// 최적화 레벨이 곡선 가속 이상 일 때
//	if (!isInlineDriveEnabled) {
//		return;
//	}
//
//	if (isLastStraight) {
//		targetInlineVal = 0;
//		return;
//	}
//
//	if (driveData[driveDataIdx].tickCnt_L > curTick_L + INLINE_RESTORE_POSITIONING_TICK
//		&& driveData[driveDataIdx].tickCnt_R > curTick_R + INLINE_RESTORE_POSITIONING_TICK) {
//
//		return;
//	}
//
//	if (driveData[driveDataIdx + 1].markState == MARK_STRAIGHT && driveData[driveDataIdx].markState == driveData[driveDataIdx + 2].markState) {
//		return;
//	}
//
//	targetInlineVal = 0;
//
//}



//__STATIC_INLINE void Inline_Drive() {
//
//	// 곡서 인라인에서 포지션을 다시 원래대로 돌려놓아야하는 길이
//	static float inlinePositioningTick_L;
//	static float inlinePositioningTick_R;
//
//	// inline이 종료되어야 하는 시점에서의 curTick
//	static float inlineEndTick_L;
//	static float inlineEndTick_R;
//
//
//	// 곡선 인라인
//	switch (curveInlineCntl) {
//
//			// 초기 상태
//			case INLINE_CNTL_IDLE :
//
//				// 최적화 레벨이 곡선 가속 이상 일 때
//				if (isInlineDriveEnabled) {
//
//						// 곡선이 36cm 미만일 때 (90도 곡선 미만일 경우)
//						//if (driveData[driveDataIdx].tickCnt_L + driveData[driveDataIdx].tickCnt_R < 2 * 0.36 * TICK_PER_M) {
//
//							inlineEndTick_L = (1 - INLINE_END_RATIO) * driveData[driveDataIdx].tickCnt_L;
//							inlineEndTick_R = (1 - INLINE_END_RATIO) * driveData[driveDataIdx].tickCnt_R;
//
//							// 곡선 막판에 인라인 재진입 방지
//							if (curTick_L < inlineEndTick_L
//							 && curTick_R < inlineEndTick_R) {
//
//
//								if (markState == MARK_CURVE_R) {
//
//									targetInlineVal = -1 * ABS_INLINE_TARGET_POSITION;
//								}
//
//								else if (markState == MARK_CURVE_L) {
//
//									targetInlineVal = ABS_INLINE_TARGET_POSITION;
//								}
//
//								curveInlineCntl = INLINE_CNTL_CURVE_IN;
//							}
//						//}
//					}
//
//
//					break ;
//
//
//			// 곡선 진입
//			case INLINE_CNTL_CURVE_IN :
//
//					// 곡선이 안전거리 미만 남았거나 curInlineVal == targetInlineVal일 경우 곡선 진입 종료
//					if (curTick_L > inlineEndTick_L
//					 || curTick_R > inlineEndTick_R
//					 || curInlineVal == targetInlineVal) {
//
//						inlinePositioningTick_L = curTick_L;
//						inlinePositioningTick_R = curTick_R;
//
//						curveInlineCntl = INLINE_CNTL_CURVE_OUT;
//					}
//
//					break;
//
//
//			// 곡선 탈출
//			case INLINE_CNTL_CURVE_OUT :
//
//					// 곡선이 (curveOutPointTick + 안전거리) 미만 남았을 경우
//					if (curTick_L > inlineEndTick_L - inlinePositioningTick_L
//					 || curTick_R > inlineEndTick_R - inlinePositioningTick_R) {
//
//						targetInlineVal = 0;
//
//						curveInlineCntl = INLINE_CNTL_END;
//					}
//
//					break ;
//
//
//
//			// 곡선 종료
//			case INLINE_CNTL_END :
//
//					// 곡선이 안전거리 미만 남았을 경우
//					if (curTick_L > inlineEndTick_L
//					 || curTick_R > inlineEndTick_R) {
//
//						curveInlineCntl = INLINE_CNTL_IDLE;
//					}
//
//					break ;
//	}
//}

#endif


