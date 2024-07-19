/*
 * second_drive.c
 */



#include "header_init.h"


__STATIC_INLINE void	Second_Drive_Cntl();
__STATIC_INLINE void	Set_Second_Drive_Data();
__STATIC_INLINE void	Second_Drive_Straight_Boost_Cntl();
__STATIC_INLINE void	Second_Drive_Curve_Inline_Cntl();




//1차 주행
void Second_Drive() {

	uint8_t	exitEcho = EXIT_ECHO_IDLE;

	uint32_t startTime = 0;
	uint32_t endTime = 0;

	uint8_t positioningIdx = 0;


	Custom_OLED_Clear();

	//주행 전 변수값 초기화
	Pre_Drive_Setting();

	Sensor_Start();
	Motor_Start();
	Speed_Control_Start();

	while (1) {

		//Drive_Test_Info_Oled();

		Positioning(&positioningIdx);

		Drive_State_Machine();
		Second_Drive_Cntl();

		//Drive_Speed_Cntl();
		if ( EXIT_ECHO_IDLE != (exitEcho = Is_Drive_End()) ) {

			Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

			while (curSpeed > DRIVE_END_DELAY_SPEED) {
				//Drive_Speed_Cntl();
			}

			Custom_Delay_ms(DRIVE_END_DELAY_TIME_MS);

			endTime = uwTick;
			break;
		}
	}

	Motor_Stop();
	Speed_Control_Stop();
	Sensor_Stop();



	Custom_OLED_Clear();

	if (exitEcho == EXIT_ECHO_END_MARK) {
		Custom_OLED_Printf("/0end mark");
		Custom_OLED_Printf("/1cross: %u", crossCnt);
		Custom_OLED_Printf("/2time: %u", endTime - startTime);
	}
	else {
		Custom_OLED_Printf("/0line out");
	}

	while (CUSTOM_SW_3 != Custom_Switch_Read());
	Custom_OLED_Clear();
}


__STATIC_INLINE void Second_Drive_Cntl() {




	switch (markState) {



		case MARK_LINE_OUT:

			break;



		case MARK_CROSS:

			// 마크 복구
			if (isReadAllMark == CUSTOM_FALSE) {

				// crossCntTable의 crossCnt 번째의 인덱스가 비어있지 않음 경우
				if (crossCntTable[crossCnt] != 0) {

					/*
					 *    n번째 크로스(crossCnt)		0		1		...		50
					 *    m번째 마크(driveDataIdx)		4(3)	6(5)	...		98
					 *
					 *    (0번째 마크에서 크로스를 읽었을 때 1번째 마크로 저장되도록 함, 0은 값이 없는 상태를 나타냄)
					 */
					driveDataIdx = crossCntTable[crossCnt] - 1;

					// 부스트, 인라인 주행 컨트롤 변수 초기화
					starightBoostCntl = BOOST_CNTL_IDLE;
					curveInlineCntl = INLINE_CNTL_IDLE;

					// isReadAllMark update
					isReadAllMark = CUSTOM_TRUE;
				}
			}

			crossCnt += 1;

			// 크로스, 엔드마크는 읽은 후 이전 상태로 되돌림
			markState = driveData[driveDataIdx].markState;

			break;




		case MARK_END:

			// 크로스, 엔드마크는 읽은 후 이전 상태로 되돌림
			markState = driveData[driveDataIdx].markState;

			break;



		case MARK_STRAIGHT:

			Set_Second_Drive_Data();

			if (isReadAllMark == CUSTOM_TRUE) {

				// 직선가속
				Second_Drive_Straight_Boost_Cntl();
			}

			break;



		case MARK_CURVE_L:
		case MARK_CURVE_R:

			Set_Second_Drive_Data();

			if (isReadAllMark == CUSTOM_TRUE) {

				// 곡선 인라인
				Second_Drive_Curve_Inline_Cntl();
			}
			break;
	}
}




__STATIC_INLINE void Set_Second_Drive_Data() {

	// markState가 변경되었을 경우
	if (markState != driveDataBuffer[driveDataIdx].markState) {

		// 현재 마크가 시작된 틱값 변경
		curTick_L = 0;
		curTick_R = 0;

		// drivePtr 값 인덱스 증가
		driveDataIdx += 1;


		// 주행중 markState와 1차 주행에서 저장된 markState가 동일하지 않다면 비정상적으로 읽었다고 판단
		if (markState != driveData[driveDataIdx].markState) {

			// 마크 인식 정상 여부를 업데이트
			isReadAllMark = CUSTOM_FALSE;

			starightBoostCntl = BOOST_CNTL_IDLE;

			targetSpeed = targetSpeed_init;

			curveInlineCntl = INLINE_CNTL_IDLE;

			targetInlineVal = 0;
		}
	}

}







__STATIC_INLINE void Second_Drive_Straight_Boost_Cntl() {

	// 감속 상수
	static float	deceleEndTickCoef;
	static float	deceleEndTickCoef_L;
	static float	deceleEndTickCoef_R;
	static float	finalDeceleEndTick_L;
	static float	finalDeceleEndTick_R;
	static uint8_t	isLastStraight = CUSTOM_FALSE;


	// 직선 가속
	switch (starightBoostCntl) {

			// 초기 상태
			case BOOST_CNTL_IDLE :

					isLastStraight = CUSTOM_FALSE;
					if (driveData[driveDataIdx + 1].markState == MARK_END) {
						isLastStraight = CUSTOM_TRUE;
					}

					// 최적화 레벨이 직선 가속 이상 일 때
					if (optimizeLevel >= OPTIMIZE_LEVEL_STRAIGHT) {


						finalDeceleEndTick_L =	driveData[driveDataIdx].tickCnt_L \
												- GET_MIN(deceleEndTick, deceleEndRatio * driveData[driveDataIdx].tickCnt_L);

						finalDeceleEndTick_R =	driveData[driveDataIdx].tickCnt_R \
												- GET_MIN(deceleEndTick, deceleEndRatio * driveData[driveDataIdx].tickCnt_R);

						starightBoostCntl = BOOST_CNTL_ACCELE;
					}

					break ;


			// 부스트 가속 컨드롤
			case BOOST_CNTL_ACCELE :

					if (curTick_L > acceleStartTick \
					 && curTick_R > acceleStartTick) {

						// deceleEndTickCoef 업데이트
						deceleEndTickCoef = 2 * decele / TICK_PER_M;

						deceleEndTickCoef_L =	deceleEndTickCoef * finalDeceleEndTick_L \
												+ targetSpeed_init * targetSpeed_init;

						deceleEndTickCoef_R =	deceleEndTickCoef * finalDeceleEndTick_R \
												+ targetSpeed_init * targetSpeed_init;

						if (isLastStraight) {
							deceleEndTickCoef_L =	deceleEndTickCoef * finalDeceleEndTick_L \
													+ LAST_STRAIGHT_TARGET_SPEED * LAST_STRAIGHT_TARGET_SPEED;

							deceleEndTickCoef_R =	deceleEndTickCoef * finalDeceleEndTick_R \
													+ LAST_STRAIGHT_TARGET_SPEED * LAST_STRAIGHT_TARGET_SPEED;
						}

						// boostSpeed로 가속
						targetSpeed = boostSpeed;

						starightBoostCntl = BOOST_CNTL_DECELE;
					}

					break;


			// 부스트 감속 컨트롤
			case BOOST_CNTL_DECELE :

					// decel이 시작되었을 경우
					/*
					 * fit_in 함수를 역산
					 * 현재 감속도를 토대로 감속하는데에 필요한 감속거리를 산출
					 * 산출된 감속거리와 실제 남은 거리를 비교
					 * 밑은 원본 식
					 *
					 * ( driveData[driveDataIdx].tickCnt - (curTick - markStartTick) - deceleEndTick ) / TICK_PER_M (실제 남은 거리)
					 * 	 <= (curSpeed * curSpeed - targetSpeed_init * targetSpeed_init) / (2 * decele) (감속도를 토대로 감속하는데에 필요한 감속거리)
					 *
					 * 위의 원본 식에서 고정된 값을 상수로 만든 뒤, 나눗셈을 없애면 식이 밑의 식이 도출됨
					 */

					if (deceleEndTickCoef_L < curSpeed * curSpeed + curTick_L * deceleEndTickCoef \
					 || deceleEndTickCoef_R < curSpeed * curSpeed + curTick_R * deceleEndTickCoef) {

						if (!isLastStraight) {
							// targetSpeed_init로 감속
							targetSpeed = targetSpeed_init;
						} else {

							targetSpeed = LAST_STRAIGHT_TARGET_SPEED;
						}

						starightBoostCntl = BOOST_CNTL_END;
					}

					break ;



			// 부스트가 종료되었을 때
			case BOOST_CNTL_END :

					// 직선이 10cm 남았을 경우
					if (curTick_L > driveData[driveDataIdx].tickCnt_L - 0.1 * TICK_PER_M \
					 || curTick_R > driveData[driveDataIdx].tickCnt_R - 0.1 * TICK_PER_M) {

						// 부스트 중 차체가 떳을 때 크로스를 못읽는 경우를 방지함
//						crossCnt = driveData[driveDataIdx].crossCnt;

						starightBoostCntl = BOOST_CNTL_IDLE;
					}

					break ;
	}
}





__STATIC_INLINE void Second_Drive_Curve_Inline_Cntl() {

	// 곡서 인라인에서 포지션을 다시 원래대로 돌려놓아야하는 길이
	static float inlinePositioningTick_L;
	static float inlinePositioningTick_R;

	// inline이 종료되어야 하는 시점에서의 curTick
	static float inlineEndTick_L;
	static float inlineEndTick_R;


	// 곡선 인라인
	switch (curveInlineCntl) {

			// 초기 상태
			case INLINE_CNTL_IDLE :

				// 최적화 레벨이 곡선 가속 이상 일 때
				if (optimizeLevel >= OPTIMIZE_LEVEL_CURVE) {

						// 곡선이 36cm 미만일 때 (90도 곡선 미만일 경우)
						//if (driveData[driveDataIdx].tickCnt_L + driveData[driveDataIdx].tickCnt_R < 2 * 0.36 * TICK_PER_M) {

							inlineEndTick_L = (1 - INLINE_END_RATIO) * driveData[driveDataIdx].tickCnt_L;
							inlineEndTick_R = (1 - INLINE_END_RATIO) * driveData[driveDataIdx].tickCnt_R;

							// 곡선 막판에 인라인 재진입 방지
							if (curTick_L < inlineEndTick_L \
							 && curTick_R < inlineEndTick_R) {


								if (markState == MARK_CURVE_R) {

									targetInlineVal = -1 * ABS_INLINE_TARGET_POSITION;
								}

								else if (markState == MARK_CURVE_L) {

									targetInlineVal = ABS_INLINE_TARGET_POSITION;
								}

								curveInlineCntl = INLINE_CNTL_CURVE_IN;
							}
						//}
					}


					break ;


			// 곡선 진입
			case INLINE_CNTL_CURVE_IN :

					// 곡선이 안전거리 미만 남았거나 curInlineVal == targetInlineVal일 경우 곡선 진입 종료
					if (curTick_L > inlineEndTick_L \
					 || curTick_R > inlineEndTick_R \
					 || curInlineVal == targetInlineVal) {

						inlinePositioningTick_L = curTick_L;
						inlinePositioningTick_R = curTick_R;

						curveInlineCntl = INLINE_CNTL_CURVE_OUT;
					}

					break;


			// 곡선 탈출
			case INLINE_CNTL_CURVE_OUT :

					// 곡선이 (curveOutPointTick + 안전거리) 미만 남았을 경우
					if (curTick_L > inlineEndTick_L - inlinePositioningTick_L \
					 || curTick_R > inlineEndTick_R - inlinePositioningTick_R) {

						targetInlineVal = 0;

						curveInlineCntl = INLINE_CNTL_END;
					}

					break ;



			// 곡선 종료
			case INLINE_CNTL_END :

					// 곡선이 안전거리 미만 남았을 경우
					if (curTick_L > inlineEndTick_L \
					 || curTick_R > inlineEndTick_R) {

						curveInlineCntl = INLINE_CNTL_IDLE;
					}

					break ;
	}
}



//
//__STATIC_INLINE void Second_Drive_Straight_Boost_Cntl() {
//
//	// 감속 상수
//	static float	deceleEndTickCoef;
//	static float	deceleEndTickCoef_L;
//	static float	deceleEndTickCoef_R;
//
//
//	// 직선 가속
//	switch (starightBoostCntl) {
//
//			// 초기 상태
//			case BOOST_CNTL_IDLE :
//
//					// 최적화 레벨이 직선 가속 이상 일 때
//					if (optimizeLevel >= OPTIMIZE_LEVEL_STRAIGHT) {
//
//						// 최소 부스트 거리 이상일 때
//						if (driveData[driveDataIdx].tickCnt_L > acceleStartTick + deceleEndTick
//						 && driveData[driveDataIdx].tickCnt_R > acceleStartTick + deceleEndTick) {
//
//							// decele 이후 다시 가속하는 것을 방지
//							if (curTick_L < markStartTick_L + driveData[driveDataIdx].tickCnt_L - deceleEndTick
//							 && curTick_R < markStartTick_R + driveData[driveDataIdx].tickCnt_R - deceleEndTick) {
//
//								// deceleEndTickCoef 업데이트
//								deceleEndTickCoef = 2 * decele / TICK_PER_M;
//
//								deceleEndTickCoef_L = deceleEndTickCoef * (markStartTick_L + driveData[driveDataIdx].tickCnt_L - deceleEndTick)
//														+ targetSpeed_init * targetSpeed_init;
//
//								deceleEndTickCoef_R = deceleEndTickCoef * (markStartTick_R + driveData[driveDataIdx].tickCnt_R - deceleEndTick)
//														+ targetSpeed_init * targetSpeed_init;
//
//								starightBoostCntl = BOOST_CNTL_ACCELE;
//							}
//						}
//					}
//
//					break ;
//
//
//			// 부스트 가속 컨드롤
//			case BOOST_CNTL_ACCELE :
//
//					// 직선 구간 진입 후 ACCELE_START_TICK만큼 지났을 때 부스트
//					if (curTick_L > markStartTick_L + acceleStartTick
//					 && curTick_R > markStartTick_R + acceleStartTick) {
//
//						// boostSpeed로 가속
//						targetSpeed = boostSpeed;
//
//						starightBoostCntl = BOOST_CNTL_DECELE;
//					}
//
//					break;
//
//
//			// 부스트 감속 컨트롤
//			case BOOST_CNTL_DECELE :
//
//					// decel이 시작되었을 경우
//					/*
//					 * fit_in 함수를 역산
//					 * 현재 감속도를 토대로 감속하는데에 필요한 감속거리를 산출
//					 * 산출된 감속거리와 실제 남은 거리를 비교
//					 * 밑은 원본 식
//					 *
//					 * ( driveData[driveDataIdx].tickCnt - (curTick - markStartTick) - deceleEndTick ) / TICK_PER_M (실제 남은 거리)
//					 * 	 <= (curSpeed * curSpeed - targetSpeed_init * targetSpeed_init) / (2 * decele) (감속도를 토대로 감속하는데에 필요한 감속거리)
//					 *
//					 * 위의 원본 식에서 고정된 값을 상수로 만든 뒤, 나눗셈을 없애면 식이 밑의 식이 도출됨
//					 */
//
//					if (deceleEndTickCoef_L <= curSpeed * curSpeed + curTick_L * deceleEndTickCoef
//					 || deceleEndTickCoef_R <= curSpeed * curSpeed + curTick_R * deceleEndTickCoef) {
//
//						// targetSpeed_init로 감속
//						targetSpeed = targetSpeed_init;
//
//						starightBoostCntl = BOOST_CNTL_END;
//					}
//
//					break ;
//
//
//
//			// 부스트가 종료되었을 때
//			case BOOST_CNTL_END :
//
//					// 직선이 10cm 남았을 경우
//					if (curTick_L > markStartTick_L + driveData[driveDataIdx].tickCnt_L - 0.1 * TICK_PER_M
//					 || curTick_R > markStartTick_R + driveData[driveDataIdx].tickCnt_R - 0.1 * TICK_PER_M) {
//
//						// 부스트 중 차체가 떳을 때 크로스를 못읽는 경우를 방지함
//						crossCnt = driveData[driveDataIdx].crossCnt;
//
//						starightBoostCntl = BOOST_CNTL_IDLE;
//					}
//
//					break ;
//	}
//}
//
//
//
//
//
//__STATIC_INLINE void Second_Drive_Curve_Inline_Cntl() {
//
//	// 곡서 인라인에서 포지션을 다시 원래대로 돌려놓아야하는 길이
//	static int32_t inlinePositioningTick_L;
//	static int32_t inlinePositioningTick_R;
//
//	// inline이 종료되어야 하는 시점에서의 curTick
//	static int32_t inlineEndTick_L;
//	static int32_t inlineEndTick_R;
//
//
//	// 곡선 인라인
//	switch (curveInlineCntl) {
//
//			// 초기 상태
//			case INLINE_CNTL_IDLE :
//
//				// 최적화 레벨이 곡선 가속 이상 일 때
//				if (optimizeLevel >= OPTIMIZE_LEVEL_CURVE) {
//
//						// 곡선이 36cm 미만일 때 (90도 곡선 미만일 경우)
//						//if (driveData[driveDataIdx].tickCnt_L + driveData[driveDataIdx].tickCnt_R < 2 * 0.36 * TICK_PER_M) {
//
//							inlineEndTick_L = INLINE_END_RATIO * driveData[driveDataIdx].tickCnt_L + markStartTick_L;
//							inlineEndTick_R = INLINE_END_RATIO * driveData[driveDataIdx].tickCnt_R + markStartTick_R;
//
//							// 곡선 막판에 인라인 재진입 방지
//							if (curTick_L < inlineEndTick_L
//							 && curTick_R < inlineEndTick_R) {
//
//
//								if (markState == MARK_CURVE_R) {
//
//									targetInlineVal =  ABS_INLINE_TARGET_POSITION;
//								}
//
//								else if (markState == MARK_CURVE_L) {
//
//									targetInlineVal = -1 * ABS_INLINE_TARGET_POSITION;
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
//						inlinePositioningTick_L = curTick_L - markStartTick_L;
//						inlinePositioningTick_R = curTick_R - markStartTick_R;
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
//

