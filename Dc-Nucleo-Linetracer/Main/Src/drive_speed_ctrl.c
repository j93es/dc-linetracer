



#include "drive_speed_ctrl.h"



// 초기의 속도 값에 관한 변수
volatile float			targetSpeed_init = TARGET_SPEED_INIT;
volatile float			accele_init = ACCELE_INIT;
volatile float			decele_init = DECELE_INIT;


// 좌우 모터 포지션에 관한 변수
volatile int32_t		positionVal = 0;
volatile float			positionCoef = POSITION_COEF_INIT;
volatile int32_t		positionTable[8] = { -14000, -10000, -6000, -2000, 2000, 6000, 10000, 14000 };



// 속도 값에 관한 변수
volatile float			accele = ACCELE_INIT;
volatile float			decele = DECELE_INIT;

volatile float			targetSpeed = TARGET_SPEED_INIT;
volatile float			curSpeed = MIN_SPEED;

volatile float			curveDeceleCoef = CURVE_DECELE_COEF_INIT;





// 가속도 및 속도 제어
void	Drive_Speed_Accele_Cntl() {


	if (curSpeed < targetSpeed) {

		// 속도 제어
		curSpeed += accele / 2000;

		if (curSpeed > targetSpeed) {

			curSpeed = targetSpeed;
		}
	}

	// curSpeed > targetSpeed 일 경우
	else {

		// 속도 제어
		curSpeed -= decele / 2000;

		if (curSpeed < targetSpeed) {

			curSpeed = targetSpeed;
		}
	}
}



// 이전 주기에서 읽은 센서 위치에서 4개만 선별
void	Make_Position_Val() {
	int32_t	positionSum = 0;
	int32_t	sensorNormValsSum = 1;


	for (int i = 1; i < 7; i++) {

		positionSum += positionTable[i] * sensorNormVals[i];
		sensorNormValsSum += sensorNormVals[i];
	}

	//divide by zero 방지하기 위해 sensorNormValsSum + 1로 나눔
	positionVal = positionSum / sensorNormValsSum;
}
