/*
 * drive_preset.c
 */

#include "header_init.h"



/*
 * 인터럽트에서 쓰는 변수
 */


// 초기의 속도 값에 관한 변수
volatile float			targetSpeed_init = TARGET_SPEED_INIT;
volatile float			targetAccele_init = ACCELE_INIT;
volatile float			decele_init = DECELE_INIT;


// 좌우 모터 포지션에 관한 변수
volatile int32_t		positionVal = 0;
volatile float			positionCoef = POSITION_COEF_INIT;
volatile int32_t		limitedPositionVal = 0;


// positionVal을 windowing하여 구하는 것에 사용되는 변수 초기화
volatile uint8_t		positionIdxMax = 5;
volatile uint8_t		positionIdxMin = 2;
volatile int32_t		positionSum = 0;
volatile int32_t		sensorNormValsSum = 0;


// 속도 값에 관한 변수
volatile float			targetAccele = ACCELE_INIT;
volatile float			curAccele = ACCELE_INIT;
volatile float			decele = DECELE_INIT;

volatile float			targetSpeed = TARGET_SPEED_INIT;
volatile float			curSpeed = MIN_SPEED;
volatile float			boostSpeed = BOOST_SPEED_INIT;

volatile float			curveDeceleCoef = CURVE_DECELE_COEF_INIT;


// 현재 모터에 몇번 상이 잡혔는 지를 카운트하는 변수
volatile uint32_t		curTick_L = 0;
volatile uint32_t		curTick_R = 0;



// 2차 주행 inline
volatile int32_t		targetInlineVal = 0;
volatile int32_t		curInlineVal = 0;





/*
 * 주행문에서 쓰는 변수
 */


// 현재 직진인지 커브인지 등을 저장하는 변수
uint8_t					markState = MARK_STRAIGHT;


// state machine 의 상태
uint8_t					driveState = DRIVE_STATE_IDLE;


// 2차주행 컨트롤 변수
uint8_t					starightBoostCntl = BOOST_CNTL_IDLE;
uint8_t					curveInlineCntl = INLINE_CNTL_IDLE;


// 2차주행에서 마크를 정확히 읽었는지 판단
uint8_t					isReadAllMark = CUSTOM_TRUE;


// driveData를 저장하고 접근하게 해주는 변수들
t_driveData				driveData[MAX_DRIVE_DATA_LEN] = { T_DRIVE_DATA_INIT, };


// 1차 주행 데이터 임시저장
t_driveData				driveDataBuffer[MAX_DRIVE_DATA_LEN] = { T_DRIVE_DATA_INIT, };


// driveData 인덱스
uint16_t				driveDataIdx = 0;


// 2차 주행에서 사용하는 cross 테이블
/*
 *    n번째 크로스(crossCnt)		0		1		...		50
 *    m번째 마크(driveDataIdx)		4(3)	6(5)	...		98
 *
 *    (0번째 마크에서 크로스를 읽었을 때 1번째 마크로 저장되도록 함, 0은 값이 없는 상태를 나타냄)
 */
uint16_t				crossCntTable[MAX_CROSS_CNT] = { 0, };


// 1차 주행에서 cross 테이블 임시 저장
uint16_t				crossCntTableBuffer[MAX_CROSS_CNT] = { 0, };


// 현재까지 읽은 크로스 개수
uint16_t				crossCnt = 0;


// 직선 주행, 곡선 인라인 최적화 레벨
uint8_t					optimizeLevel = OPTIMIZE_LEVEL_NONE;


// 현재 마크가 시작된 tick
uint32_t				markStartTick_L = 0;
uint32_t				markStartTick_R = 0;


//end mark를 몇 번 봤는지 카운트하는 변수
uint8_t					endMarkCnt = 0;


// 피트인 거리
float					pitInLen = PIT_IN_LEN_INIT;


// state machine 에서 사용
//센서 값 누적
uint8_t					sensorStateSum = 0x00;


// 2차 주행 직선가속에서 사용
float					acceleStartTick = ACCELE_START_TICK_INIT;

// 안전 거리
float					deceleEndTick = DECELE_END_TICK_INIT;

// 안전 비율
float					deceleEndRatio = DECELE_END_RATIO_INIT;





// 주행 전 초기값 조정
static void Pre_Drive_Var_Adjust_First_Drive();
static void Pre_Drive_Var_Adjust_Second_Drive();
static void Pre_Drive_Var_Adjust_Switch_Cntl(t_driveMenu_Int *intValues, t_driveMenu_Float *floatValues, \
											uint8_t intValCnt, uint8_t floatValCnt, uint8_t isEnd);

// 주행 전 초기값 대입
static void Pre_Drive_Var_Init();





//주행 전 상수값 변경 절차
void Pre_Drive_Setting() {

	if (optimizeLevel >= OPTIMIZE_LEVEL_STRAIGHT) {
		Pre_Drive_Var_Adjust_Second_Drive();
	}

	if (optimizeLevel >= OPTIMIZE_LEVEL_NONE) {
		Pre_Drive_Var_Adjust_First_Drive();
	}

	Pre_Drive_Var_Init();

}



// 주행 전 초기값 조정
static void Pre_Drive_Var_Adjust_First_Drive() {

	t_driveMenu_Int		intValues[] = {

			{ "Threshold",			&threshold,			10 },
	};
	uint8_t intValCnt = sizeof(intValues) / sizeof(t_driveMenu_Int);


	t_driveMenu_Float	floatValues[] = {

			{ "Pit In Len",			&pitInLen,			0.01f },
			{ "Target Speed",		&targetSpeed_init,	0.05f },
			{ "CurveDecel Coef",	&curveDeceleCoef,	500 },
			//{ "Position Coef",		&positionCoef,		0.000001f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);



	Pre_Drive_Var_Adjust_Switch_Cntl(intValues, floatValues, intValCnt, floatValCnt, CUSTOM_TRUE);
}



static void Pre_Drive_Var_Adjust_Second_Drive() {

	float acceleStartLen = acceleStartTick / TICK_PER_M;
	float deceleEndLen = deceleEndTick / TICK_PER_M;

	t_driveMenu_Int		intValues[] = {

			{ "optimize level",		&optimizeLevel,		1 },
	};
	uint8_t intValCnt = sizeof(intValues) / sizeof(t_driveMenu_Int);


	t_driveMenu_Float	floatValues[] = {

			{ "Boost Speed",		&boostSpeed,		0.25f },
			{ "Accele",				&targetAccele_init,	0.25f },
			//{ "Decele",				&decele_init,		0.25f },
			{ "acceleStart len",	&acceleStartLen,	0.025f },
			{ "decelEnd len",		&deceleEndLen,		0.05f },
			{ "decelEnd ratio",		&deceleEndRatio,	0.05f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);



	Pre_Drive_Var_Adjust_Switch_Cntl(intValues, floatValues, intValCnt, floatValCnt, CUSTOM_FALSE);

	acceleStartTick = acceleStartLen * TICK_PER_M;
	deceleEndTick = deceleEndLen * TICK_PER_M;

}


static void Pre_Drive_Var_Adjust_Switch_Cntl(t_driveMenu_Int *intValues, t_driveMenu_Float *floatValues, \
											uint8_t intValCnt, uint8_t floatValCnt, uint8_t isEnd) {

	uint8_t	sw = 0;


	for (uint8_t i = 0; i < intValCnt; i++) {

		Custom_OLED_Clear();

		// 정수 변수 초기화
		if (i < intValCnt) {

			while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

				// OLED에 변수명 변수값 출력
				Custom_OLED_Printf("/2%s", intValues[i].valName);
				Custom_OLED_Printf("/A/4%5d", *(intValues[i].val));

				// 변수 값 빼기
				if (sw == CUSTOM_SW_1) {
					*(intValues[i].val) -= intValues[i].changeVal;
				}
				// 변수값 더하기
				else if (sw == CUSTOM_SW_2) {
					*(intValues[i].val) += intValues[i].changeVal;
				}
			}
		}
	}


	for (uint8_t i = 0; i < floatValCnt; i++) {

		Custom_OLED_Clear();

		while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

			uint32_t num1 = (uint32_t)(*(floatValues[i].val));
			uint32_t num2 = (uint32_t)( *(floatValues[i].val) * 100000 - num1 * 100000 );

			// OLED에 변수명 변수값 출력
			Custom_OLED_Printf("/2%s", floatValues[i].valName);
			Custom_OLED_Printf("/A/4%u.%05u", num1, num2);

			if (isEnd == CUSTOM_TRUE && i == floatValCnt - 1) {
				Custom_OLED_Printf("/g/0Ready to Drive");
			}

			// 변수 값 빼기
			if (sw == CUSTOM_SW_1) {
				*(floatValues[i].val) -= floatValues[i].changeVal;
			}
			// 변수값 더하기
			else if (sw == CUSTOM_SW_2) {
				*(floatValues[i].val) += floatValues[i].changeVal;
			}
		}
	}

	Custom_OLED_Clear();
}



// 주행 전 초기값 대입
static void Pre_Drive_Var_Init() {


	/*
	 * 인터럽트에서 쓰는 변수
	 */

	// 가속도 변수 초기화
	targetAccele = targetAccele_init;
	curAccele = 0;

	// 속도 관련 변수 초기화
	targetSpeed = targetSpeed_init;
	decele = decele_init;
	curSpeed = MIN_SPEED;

	// 좌우모터 포지션 값을 0으로 초기화
	positionVal = 0;
	limitedPositionVal = 0;

	// positionVal을 windowing하여 구하는 것에 사용되는 변수 초기화
	positionIdxMax = 5;
	positionIdxMin = 2;
	positionSum = 0;
	sensorNormValsSum = 0;

	// 현재 모터가 상을 잡은 횟수 초기화
	curTick_L = 0;
	curTick_R = 0;

	// 2차 주행 inline 관현 값 초기화
	targetInlineVal = 0;
	curInlineVal = 0;



	/*
	 * 주행문에서 쓰는 변수
	 */

	// 현재 마크 인식 상태를 직선 주행으로 초기화
	markState = MARK_STRAIGHT;

	// state machine 의 상태 업데이트
	driveState = DRIVE_STATE_IDLE;

	// 현재까지 읽은 크로스 개수 업데이트
	crossCnt = 0;

	// 현재 마크가 시작된 tick
	markStartTick_L = 0;
	markStartTick_R = 0;

	// 엔드마크 읽은 개수 초기화
	endMarkCnt = 0;

	// driveData 인덱스 초기화
	driveDataIdx = 0;


	// 1차 주행에서만 초기화할 변수
	if (optimizeLevel == OPTIMIZE_LEVEL_NONE) {

		optimizeLevel = OPTIMIZE_LEVEL_NONE;

		for (uint32_t i = 0; i < MAX_DRIVE_DATA_LEN; i++) {
			t_driveData temp = T_DRIVE_DATA_INIT;

			driveDataBuffer[i] = temp;
		}

		// driveData의 0, 1번째 값 초기화
		// 0번 인덱스는 할당되지 않은 포인터에 접근하지 않도록 고정시켜둠
		// 실질적으로 주행은 1번 인덱스부터 시작
		driveDataBuffer[0].markState = MARK_STRAIGHT;


		for (uint32_t i = 0; i < MAX_CROSS_CNT; i++) {

			crossCntTableBuffer[i] = 0;
		}
	}

	// 2, 3차 주행에서만 초기화할 변수
	if (optimizeLevel >= OPTIMIZE_LEVEL_STRAIGHT) {

		// isReadAllMark 값 정상으로 변경
		isReadAllMark = CUSTOM_TRUE;

		// 부스트 컨트롤 상태 업데이트
		starightBoostCntl = BOOST_CNTL_IDLE;
	}

	// 3차 주행에서만 초기화할 변수
	if (optimizeLevel >= OPTIMIZE_LEVEL_CURVE){

		// 커브 인라인 상태 업데이트
		curveInlineCntl = INLINE_CNTL_IDLE;
	}
}




void	Time_Attack_Setting() {


	t_driveMenu_Int		intValues[] = {

			{ "Threshold",			&threshold,			5 },
	};
	uint8_t intValCnt = sizeof(intValues) / sizeof(t_driveMenu_Int);


	t_driveMenu_Float	floatValues[] = {

			{ "Pit In Len",			&pitInLen,			0.01f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);

	targetAccele_init = 2.f;
	positionCoef = POSITION_COEF_INIT;
	curveDeceleCoef = CURVE_DECELE_COEF_INIT;


	Pre_Drive_Var_Adjust_Switch_Cntl(intValues, floatValues, intValCnt, floatValCnt, CUSTOM_TRUE);

	Pre_Drive_Var_Init();
}


