/*
 * first_drive.c
 */

#include "drive_preset.h"
#include "drive_speed_ctrl.h"
#include "first_drive.h"
#include "motor.h"
#include "sensor.h"

#include "custom_oled.h"
#include "custom_switch.h"





// 현재 직진인지 커브인지 등을 저장하는 변수
uint8_t					markState = MARK_STRAIGHT;


// state machine 의 상태
uint8_t					driveState = DRIVE_STATE_IDLE;


//end mark를 몇 번 봤는지 카운트하는 변수
uint8_t					endMarkCnt = 0;


// 피트인 거리
float					pitInLen = PIT_IN_LEN_INIT;


// state machine 에서 사용
//센서 값 누적
uint8_t					sensorStateSum = 0x00;




// line sensor가 읽은 값을 개수를 리턴함
uint8_t	Get_Line_Sensor_Cnt() {
	return ((state >> 6) & 0x01) + ((state >> 5) & 0x01) + ((state >> 4) & 0x01) + \
			((state >> 3) & 0x01) + ((state >> 2) & 0x01) + ((state >> 1) & 0x01);
}


// marker sensor가 읽은 값을 개수를 리턴함
uint8_t	Get_Marker_Sensor_Cnt() {
	return ((state >> 7) & 0x01) + ((state >> 0) & 0x01);
}




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


		/*
		 * 마크 정보, 이동한 거리(틱수) 저장
		 * 개인이 알아서 작성하도록 지시
		 * */

		//Drive_Speed_Cntl();
		if ( EXIT_ECHO_IDLE != (exitEcho = Is_Drive_End(exitEcho)) ) {

			endTime = uwTick;
			break;
		}
	}

	Motor_Stop();
	Speed_Control_Stop();
	Sensor_Stop();

	Custom_OLED_Printf("%u", endTime - startTime);
}







void	Drive_State_Machine() {

	static uint32_t	lineOutStartTime;


	switch (driveState) {


		case DRIVE_STATE_IDLE :

				// 라인 센서 4개 이상 인식
				if (Get_Line_Sensor_Cnt() >= 4) {

					sensorStateSum = 0x00;

					driveState = DRIVE_STATE_CROSS;
				}

				// 라인 센서 4개 이하 and 마크 센서 1개 이상
				else if (Get_Marker_Sensor_Cnt() != 0) {

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
				if (sensorStateSum == 0xff && Get_Marker_Sensor_Cnt() == 0) {

					driveState = DRIVE_STATE_DECISION;
				}

				break;





		case DRIVE_STATE_MARKER :

				// accum
				sensorStateSum |= state;

				// 마커 센서가 0개 일 때
				if (Get_Marker_Sensor_Cnt() == 0) {

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





// end line, right mark, left mark, straight를 판별하고 정해진 동작을 실행하는 함수
void	Decision(uint8_t sensorStateSum) {


	// cross
	if (sensorStateSum == 0xff) {

		markState = MARK_CROSS;
	}


	// end mark
	// if ( ((sensorStateSum >> 0) & 0x01) && ((sensorStateSum >> 7) & 0x01) )
	else if ( (sensorStateSum & 0x81) == 0x81 ) {

		markState = MARK_END;
		endMarkCnt += 1;
	}


	// left mark
	else if ( (sensorStateSum & 0x80) == 0x80 ) {


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
	else if ( (sensorStateSum & 0x01) == 0x01 ) {

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





// 피트인 함수
void	Drive_Fit_In(float s, float pinSpeed) {

	targetSpeed = pinSpeed;
	decele = ABS( (pinSpeed - curSpeed) * (pinSpeed + curSpeed) ) / (2 * s);
}





uint8_t	Is_Drive_End(uint8_t exitEcho) {

	// endMark || lineOut
	if (endMarkCnt >= 2 || markState == MARK_LINE_OUT) {

		Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

		while (curSpeed > DRIVE_END_DELAY_SPEED) {
			//Drive_Speed_Cntl();
		}

		Custom_Delay_ms(DRIVE_END_DELAY_TIME_MS);

		if (endMarkCnt >= 2) {

			exitEcho = EXIT_ECHO_END_MARK;
		}
		else {

			exitEcho = EXIT_ECHO_LINE_OUT;
		}
	}

	return exitEcho;
}
