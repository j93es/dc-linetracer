/*
 * second_drive.c
 */



#include "header_init.h"


__STATIC_INLINE void	Second_Drive_Cntl();
__STATIC_INLINE void	Set_Second_Drive_Data();




//2차 주행
void Second_Drive() {

	uint8_t	exitEcho = EXIT_ECHO_IDLE;

	uint32_t startTime = 0;
	uint32_t endTime = 0;

	uint8_t positioningIdx = 0;


	Custom_OLED_Clear();

	Drive_Optimize_Setting();

	//주행 전 변수값 초기화
	Pre_Drive_Setting();

	Sensor_Start();
	Motor_Start();
	Speed_Control_Start();

	while (1) {

		//Drive_Test_Info_Oled();

		Positioning(&positioningIdx);

		Mark();
		Second_Drive_Cntl();

		//Drive_Speed_Cntl();
		if ( EXIT_ECHO_IDLE != (exitEcho = Is_Drive_End(exitEcho)) ) {

			Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

			while (curSpeed > DRIVE_END_DELAY_SPEED) {
				Positioning(&positioningIdx);
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
	}
	else {
		Custom_OLED_Printf("/0line out");
	}

	Custom_OLED_Printf("/1cross: %u", crossCnt);

	Custom_OLED_Printf("%u", endTime - startTime);

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
					curveBoostCntl = BOOST_CNTL_IDLE;
					curveInlineCntl = INLINE_CNTL_IDLE;

					// isReadAllMark update
					isReadAllMark = CUSTOM_TRUE;
				}
			}

			crossCnt += 1;

			// 크로스, 엔드마크는 읽은 후 이전 상태로 되돌림
//			markState = driveDataBuffer[driveDataIdx].markState;

			// 크로스, 엔드마크는 읽은 후 마커를 강제로 직선으로 변경
			markState = MARK_STRAIGHT;
			driveDataBuffer[driveDataIdx].markState = MARK_STRAIGHT;

			break;




		case MARK_END:

			endMarkCnt++;

			// 크로스, 엔드마크는 읽은 후 이전 상태로 되돌림
//			markState = driveDataBuffer[driveDataIdx].markState;

			// 크로스, 엔드마크는 읽은 후 마커를 강제로 직선으로 변경
			markState = MARK_STRAIGHT;
			driveDataBuffer[driveDataIdx].markState = MARK_STRAIGHT;


			break;



		case MARK_STRAIGHT:

			Set_Second_Drive_Data();

			if (isReadAllMark == CUSTOM_TRUE) {

				// 직선가속
				Straight_Boost();

				Prepare_Inline();
			}

			break;



		case MARK_CURVE_L:
		case MARK_CURVE_R:

			Set_Second_Drive_Data();


			if (isReadAllMark == CUSTOM_TRUE) {

				Curve_Boost();

				// 곡선 인라인
				Restore_Inline();
			}
			break;
	}
}




__STATIC_INLINE void Set_Second_Drive_Data() {

	// markState가 변경되었을 경우
	if (markState != driveDataBuffer[driveDataIdx].markState) {




		// 현재마크에서 이동한 tick 값을 현재 인덱스의 구조체에 저장
		driveDataBuffer[driveDataIdx].tickCnt_L = curTick_L;
		driveDataBuffer[driveDataIdx].tickCnt_R = curTick_R;

		// curTick 초기화
		curTick_L = 0;
		curTick_R = 0;

		// 종료 시점에서의 읽은 크로스의 개수
		driveDataBuffer[driveDataIdx].crossCnt = crossCnt;

		// drivePtr 값 인덱스 증가
		driveDataIdx += 1;


		// 증가된 구조체의 인덱스에 markState 저장
		driveDataBuffer[driveDataIdx].markState = markState;





		isLastStraight = CUSTOM_FALSE;
		if (driveData[driveDataIdx].markState == MARK_END) {
			isLastStraight = CUSTOM_TRUE;
		}



		starightBoostCntl = BOOST_CNTL_IDLE;
		curveBoostCntl = BOOST_CNTL_IDLE;
		curveInlineCntl = INLINE_CNTL_IDLE;
		targetSpeed = targetSpeed_init;




		// 주행중 markState와 1차 주행에서 저장된 markState가 동일하지 않다면 비정상적으로 읽었다고 판단
		if (markState != driveData[driveDataIdx].markState) {

			// 마크 인식 정상 여부를 업데이트
			isReadAllMark = CUSTOM_FALSE;

//			starightBoostCntl = BOOST_CNTL_IDLE;
//
//			targetSpeed = targetSpeed_init;
//
//			curveInlineCntl = INLINE_CNTL_IDLE;

			targetInlineVal = 0;
		}
	}

}

