/*
 * drive_preset.c
 */

#include "header_init.h"



// 주행 전 초기값 조정
static void Adjust_First_Drive();
static void Adjust_Straight_Boost();
static void Adjust_Curve_Boost();


static void Adjust_Int_Val(t_driveMenu_Int *intValues, uint8_t intValCnt, uint8_t isEnd);
static void Adjust_Float_Val(t_driveMenu_Float *floatValues, uint8_t intValCnt, uint8_t isEnd);




void Drive_Optimize_Setting() {
	t_driveMenu_Int		intValues[] = {
			{ "straight boost",			&isStraightBoostEnabled,		1 },
			{ "curve boost   ",			&isCurveBoostEnabled,			1 },
			{ "inline drive  ",			&isInlineDriveEnabled,			1 },
	};
	uint8_t intValCnt = sizeof(intValues) / sizeof(t_driveMenu_Int);
	Adjust_Int_Val(intValues, intValCnt, CUSTOM_FALSE);
}




//주행 전 상수값 변경 절차
void Pre_Drive_Setting() {

	if (isStraightBoostEnabled) {
		Adjust_Straight_Boost();
	}

	if (isCurveBoostEnabled) {
		Adjust_Curve_Boost();
	}

	Adjust_First_Drive();

	Pre_Drive_Var_Init();

}



// 주행 전 초기값 조정
static void Adjust_First_Drive() {

	t_driveMenu_Int		intValues[] = {

			{ "Threshold",			&threshold,			10 },
	};
	uint8_t intValCnt = sizeof(intValues) / sizeof(t_driveMenu_Int);
	Adjust_Int_Val(intValues, intValCnt, CUSTOM_FALSE);


	t_driveMenu_Float	floatValues[] = {

			{ "Pit In Len",			&pitInLen,			0.01f },
			{ "Target Speed",		&targetSpeed_init,	0.05f },
			{ "CurveDecel Coef",	&curveDeceleCoef,	500 },
			{ "Position Coef",		&positionCoef,		0.000001f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);
	Adjust_Float_Val(floatValues, floatValCnt, CUSTOM_TRUE);
}



static void Adjust_Straight_Boost() {

	float acceleStartLen = acceleStartTick / TICK_PER_M;
	float deceleEndLen = deceleEndTick / TICK_PER_M;

	t_driveMenu_Float	floatValues[] = {

			{ "st boost V",		&starightBoostSpeed,	0.25f },
			{ "Accele",				&targetAccele_init,	0.25f },
			//{ "Decele",				&decele_init,		0.25f },
			{ "acceleStart len",	&acceleStartLen,	0.025f },
			{ "decelEnd len",		&deceleEndLen,		0.05f },
			{ "decelEnd ratio",		&deceleEndRatio,	0.05f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);
	Adjust_Float_Val(floatValues, floatValCnt, CUSTOM_TRUE);


	acceleStartTick = acceleStartLen * TICK_PER_M;
	deceleEndTick = deceleEndLen * TICK_PER_M;

}


static void Adjust_Curve_Boost() {


	t_driveMenu_Float	floatValues[] = {

			{ "cu boost V",		&curveBoostSpeed,		0.25f },
	};
	uint8_t floatValCnt = sizeof(floatValues) / sizeof(t_driveMenu_Float);
	Adjust_Float_Val(floatValues, floatValCnt, CUSTOM_TRUE);
}



static void Adjust_Int_Val(t_driveMenu_Int *intValues, uint8_t intValCnt, uint8_t isEnd) {

	uint8_t	sw = 0;

	for (uint8_t i = 0; i < intValCnt; i++) {

		Custom_OLED_Clear();

		// 정수 변수 초기화
		if (i < intValCnt) {

			while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

				// OLED에 변수명 변수값 출력
				Custom_OLED_Printf("/2%s", intValues[i].valName);
				Custom_OLED_Printf("/A/4%5d", *(intValues[i].val));

				if (isEnd == CUSTOM_TRUE && i == intValCnt - 1) {
					Custom_OLED_Printf("/g/0Ready to Drive");
				}

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

	Custom_OLED_Clear();

}

static void Adjust_Float_Val(t_driveMenu_Float *floatValues, uint8_t floatValCnt, uint8_t isEnd) {

	uint8_t	sw = 0;

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
void Pre_Drive_Var_Init() {


	/*
	 * 인터럽트에서 쓰는 변수
	 */

	// pd 제어에 사용하는 변수 초기화
	levelMaxCCR = TIM10->ARR + 1;
	positionCmdL = 0;
	positionL = 0;
	positionCmdR = 0;
	positionR = 0;
//	prevErrorL = 0;
//	prevErrorR = 0;
//	targetEncoderValueL_cntl = 0;
//	targetEncoderValueR_cntl = 0;
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	prevEncoderValueL = 0;
	prevEncoderValueR = 0;
	pCoef = P_COEF_INIT;
	dCoef = D_COEF_INIT;

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
	markStateMachine = MARK_STATE_MACHINE_IDLE;

	// 현재까지 읽은 크로스 개수 업데이트
	crossCnt = 0;

	// 엔드마크 읽은 개수 초기화
	endMarkCnt = 0;

	// driveData 인덱스 초기화
	driveDataIdx = 0;


	lineMasking = LINE_MASKING_INIT;
	rightMarkMasking = RIGHT_MARK_MASKING_INIT;
	leftMarkMasking = LEFT_MARK_MASKING_INIT;
	bothMarkMasking = RIGHT_MARK_MASKING_INIT | LEFT_MARK_MASKING_INIT;
	markAreaMasking = ~(lineMasking << 1 | lineMasking >> 1);

	// isReadAllMark 값 정상으로 변경
	isReadAllMark = CUSTOM_TRUE;

	// 부스트 컨트롤 상태 업데이트
	starightBoostCntl = BOOST_CNTL_IDLE;

	curveBoostCntl = BOOST_CNTL_IDLE;

	// 커브 인라인 상태 업데이트
	curveInlineCntl = INLINE_CNTL_IDLE;

	isLastStraight = CUSTOM_FALSE;


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


