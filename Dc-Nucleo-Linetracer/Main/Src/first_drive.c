/*
 * first_drive.c
 */

#include "header_init.h"



__STATIC_INLINE void	First_Drive_Ctrl();
__STATIC_INLINE void	Set_First_Drive_Data();
static void				First_Drive_Data_Cntl(uint8_t exitEcho);
static void				First_Drive_Data_Update_Cntl(uint8_t exitEcho);




//1차 주행
void First_Drive() {

	uint8_t exitEcho = EXIT_ECHO_IDLE;

	uint32_t startTime = 0;
	uint32_t endTime = 0;

	Custom_OLED_Clear();

	//주행 전 변수값 초기화
	Pre_Drive_Setting();

	Sensor_Start();
	Motor_Start();
	Speed_Control_Start();

	startTime = uwTick;

	while (1) {

		//Drive_Test_Info_Oled();

		Drive_State_Machine();
		First_Drive_Ctrl();

		//Drive_Speed_Cntl();
		if ( EXIT_ECHO_IDLE != (exitEcho = Is_Drive_End(exitEcho)) ) {

			endTime = uwTick;
			break;
		}
	}

	Motor_Stop();
	Speed_Control_Stop();
	Sensor_Stop();


	First_Drive_Data_Cntl(exitEcho);

	Custom_OLED_Printf("%u", endTime - startTime);
}






__STATIC_INLINE void First_Drive_Ctrl() {

	if (markState == MARK_LINE_OUT) {

		return ;
	}

	// markState가 변경되었을 경우
	else if (markState != driveDataBuffer[driveDataIdx].markState) {

		// driveData 값 업데이트
		Set_First_Drive_Data();
	}
}



__STATIC_INLINE void Set_First_Drive_Data() {

	// 크로스, 엔드마크가 아닐 경우
	if (markState != MARK_CROSS && markState != MARK_END) {

		// 현재마크에서 이동한 tick 값을 현재 인덱스의 구조체에 저장
		driveDataBuffer[driveDataIdx].tickCnt_L = curTick_L - markStartTick_L;
		driveDataBuffer[driveDataIdx].tickCnt_R = curTick_R - markStartTick_R;

		// 종료 시점에서의 읽은 크로스의 개수
		driveDataBuffer[driveDataIdx].crossCnt = crossCnt;

		// drivePtr 값 인덱스 증가
		driveDataIdx += 1;

		// markStartTick 업데이트
		markStartTick_L = curTick_L;
		markStartTick_R = curTick_R;


		// 증가된 구조체의 인덱스에 markState 저장
		driveDataBuffer[driveDataIdx].markState = markState;
	}

	else {

		// 크로스일 경우
		if (markState == MARK_CROSS) {

			/*
			 *    n번째 크로스(crossCnt)		0		1		...		50
			 *    m번째 마크(driveDataIdx)		4(3)	6(5)	...		98
			 *
			 *    (0번째 마크에서 크로스를 읽었을 때 1번째 마크로 저장되도록 함, 0은 값이 없는 상태를 나타냄)
			 */
			crossCntTableBuffer[crossCnt] = driveDataIdx + 1;

			crossCnt += 1;
		}

		// 엔드마크일 경우
		else if (markState == MARK_END){

			endMarkCnt += 1;

			if (endMarkCnt >= 2) {

				// 현재마크에서 이동한 tick 값을 현재 인덱스의 구조체에 저장
				driveDataBuffer[driveDataIdx].tickCnt_L = curTick_L - markStartTick_L;
				driveDataBuffer[driveDataIdx].tickCnt_R = curTick_R - markStartTick_R;

				// 종료 시점에서의 읽은 크로스의 개수
				driveDataBuffer[driveDataIdx].crossCnt = crossCnt;
			}
		}

		// 크로스, 엔드마크는 읽은 후 이전 상태로 되돌림
		markState = driveDataBuffer[driveDataIdx].markState;
	}

}






static void First_Drive_Data_Cntl(uint8_t exitEcho) {
	uint32_t i = 1;
	uint16_t markCnt_L = 0;
	uint16_t markCnt_R = 0;
	uint16_t crossCnt = 0;

	if (exitEcho == EXIT_ECHO_END_MARK) {

		// 마크 개수 세기
		for (i = 1; driveDataBuffer[i].markState != MARK_NONE && i < MAX_DRIVE_DATA_LEN; i++) {

			// 현재상태가 좌측 곡선인 경우
			if (driveDataBuffer[i].markState == MARK_CURVE_L) {

				markCnt_L += 1;
			}

			// 현재상태가 우측 곡선인 경우
			else if (driveDataBuffer[i].markState == MARK_CURVE_R) {

				markCnt_R += 1;
			}

			// 직선 (인덱스가 1부터 시작하기에 지정되지 않은 메모리에 접근하는 행동을 방지함)
			else if (driveDataBuffer[i].markState == MARK_STRAIGHT) {

				// 이전 상태가 좌측 곡선이었을 경우
				if (driveDataBuffer[i-1].markState == MARK_CURVE_L) {
					markCnt_L += 1;
				}

				// 이전 상태가 우측 곡선이었을 경우
				else if (driveDataBuffer[i-1].markState == MARK_CURVE_R) {
					markCnt_R += 1;
				}
			}
		}

		for (i = 0; crossCntTableBuffer[i] != 0 && i < MAX_CROSS_CNT; i++) {

			crossCnt++;
		}

		Custom_OLED_Clear();

		// OLED에 exitEcho 변수명 및 마크 개수 출력
		Custom_OLED_Printf("/0end mark");
		Custom_OLED_Printf("/1mark L:   %d", markCnt_L);
		Custom_OLED_Printf("/2mark R:   %d", markCnt_R);
		Custom_OLED_Printf("/3cross:    %d", crossCnt);

		while (CUSTOM_SW_3 != Custom_Switch_Read()) ;

		First_Drive_Data_Update_Cntl(exitEcho);
	}

	else if (exitEcho == EXIT_ECHO_LINE_OUT){

		Custom_OLED_Printf("/0line out");

		while (CUSTOM_SW_3 != Custom_Switch_Read()) ;
	}

	Custom_OLED_Clear();
}



static void First_Drive_Data_Update_Cntl(uint8_t exitEcho) {

	uint8_t sw;
	uint8_t isUpdate = CUSTOM_FALSE;

	Custom_OLED_Printf("/5update: NO");

	while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

		// data 업데이트 함
		if (sw == CUSTOM_SW_1) {
			Custom_OLED_Printf("/5update: YES");
			isUpdate = CUSTOM_TRUE;
		}

		// data 업데이트 안함
		else if (sw == CUSTOM_SW_2) {
			Custom_OLED_Printf("/5update: NO ");
			isUpdate = CUSTOM_FALSE;
		}
	}
	Custom_OLED_Clear();

	if (driveData[0].markState == MARK_NONE || isUpdate == CUSTOM_TRUE) {

		for (uint32_t i = 0; i < MAX_DRIVE_DATA_LEN; i++) {
			driveData[i].tickCnt_L = driveDataBuffer[i].tickCnt_L;
			driveData[i].tickCnt_R = driveDataBuffer[i].tickCnt_R;
			driveData[i].markState = driveDataBuffer[i].markState;
			driveData[i].crossCnt = driveDataBuffer[i].crossCnt;
		}

		for (uint32_t i = 0; i < MAX_CROSS_CNT; i++) {

			crossCntTable[i] = crossCntTableBuffer[i];
		}
	}
}






void Drive_Time_Attack_Get_Data() {

	Custom_OLED_Clear();

	//주행 전 변수값 초기화
	Time_Attack_Setting();

	Sensor_Start();
	Motor_Start();
	Speed_Control_Start();

	while (1) {

		//Drive_Test_Info_Oled();

		Drive_State_Machine();
		First_Drive_Ctrl();

		//Drive_Speed_Cntl();
		if (endMarkCnt >= 2 || markState == MARK_LINE_OUT) {

			Custom_OLED_Printf("/0%u", curTick_L);

			Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

			while (curSpeed > DRIVE_END_DELAY_SPEED) {
				//Drive_Speed_Cntl();
			}

			break;
		}
	}

	Motor_Stop();
	Speed_Control_Stop();
	Sensor_Stop();

	while (CUSTOM_SW_3 != Custom_Switch_Read()) ;

	Custom_OLED_Clear();
}





void Drive_Time_Attack() {
	uint8_t	sw = 0;
	uint8_t exitEcho = EXIT_ECHO_IDLE;
	int32_t targetTick = 0;
	int32_t targetMs = 0;
	int32_t pitInMs = 0;


	//주행 전 변수값 초기화
	Time_Attack_Setting();



	for (int i = 9;  i > 0; i--) {


		while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

			Custom_OLED_Printf("/0target tick");

			// n 번째 자리수인지 출력
			Custom_OLED_Printf("/1%d", i);

			// OLED에 변수값 출력
			Custom_OLED_Printf("/2%d", targetTick);


			// 변수 값 빼기
			if (sw == CUSTOM_SW_1) {
				targetTick -= 1;
			}
			// 변수값 더하기
			else if (sw == CUSTOM_SW_2) {
				targetTick += 1;
			}
		}

		targetTick = 10 * targetTick;
	}

	Custom_OLED_Clear();




	for (int i = 5;  i > 0; i--) {

		while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

			Custom_OLED_Printf("/0target ms");

			// n 번째 자리수인지 출력
			Custom_OLED_Printf("/1%d", i);

			// OLED에 변수값 출력
			Custom_OLED_Printf("/2%d", targetMs);


			// 변수 값 빼기
			if (sw == CUSTOM_SW_1) {
				targetMs -= 1;
			}
			// 변수값 더하기
			else if (sw == CUSTOM_SW_2) {
				targetMs += 1;
			}
		}

		targetMs = 10 * targetMs;
	}


	Custom_OLED_Clear();




	for (int i = 5;  i > 0; i--) {

		while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

			Custom_OLED_Printf("/0pit in ms");

			// n 번째 자리수인지 출력
			Custom_OLED_Printf("/1%d", i);

			// OLED에 변수값 출력
			Custom_OLED_Printf("/2%d", pitInMs);


			// 변수 값 빼기
			if (sw == CUSTOM_SW_1) {
				pitInMs -= 1;
			}
			// 변수값 더하기
			else if (sw == CUSTOM_SW_2) {
				pitInMs += 1;
			}
		}
		pitInMs = 10 * pitInMs;
	}







	Custom_OLED_Clear();

	Sensor_Start();
	Motor_Start();
	Speed_Control_Start();

	while (1) {

		//Drive_Test_Info_Oled();

		Drive_State_Machine();
		First_Drive_Ctrl();

		//Drive_Speed_Cntl();
		if ( curTick_L > targetTick - 0.5 * TICK_PER_M || endMarkCnt >= 2 || markState == MARK_LINE_OUT ) {

			if (endMarkCnt >= 2 || markState == MARK_LINE_OUT) {

				exitEcho = ~EXIT_ECHO_IDLE;
			}

			Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

			while (curSpeed > DRIVE_END_DELAY_SPEED) {
				//Drive_Speed_Cntl();
			}

			break;
		}
	}

	Motor_Stop();
	Speed_Control_Stop();




	if (exitEcho == EXIT_ECHO_IDLE) {

		while (uwTick < targetMs - pitInMs) ;

		targetSpeed = 1.5f;


		Motor_Start();
		Speed_Control_Start();

		while (1) {

			//Drive_Test_Info_Oled();

			Drive_State_Machine();
			First_Drive_Ctrl();

			//Drive_Speed_Cntl();
			if ( endMarkCnt >= 2 || markState == MARK_LINE_OUT ) {

				Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

				while (curSpeed > DRIVE_END_DELAY_SPEED) {
					//Drive_Speed_Cntl();
				}

				break;
			}
		}

		Motor_Stop();
		Speed_Control_Stop();
		Sensor_Stop();

		Custom_OLED_Clear();
	}
}

