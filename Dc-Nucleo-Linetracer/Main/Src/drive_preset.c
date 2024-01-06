/*
 * drive_preset.c
 */

#include "drive_preset.h"
#include "drive_speed_ctrl.h"
#include "first_drive.h"
#include "motor.h"
#include "sensor.h"


#include "custom_gpio.h"
#include "custom_oled.h"
#include "custom_switch.h"





// 주행 전 초기값 조정
static void Pre_Drive_Var_Adjust_First_Drive();
static void Pre_Drive_Var_Adjust_Switch_Cntl(t_driveMenu_Int *intValues, t_driveMenu_Float *floatValues, \
											uint8_t intValCnt, uint8_t floatValCnt, uint8_t isEnd);

// 주행 전 초기값 대입
static void Pre_Drive_Var_Init();





//주행 전 상수값 변경 절차
void Pre_Drive_Setting() {


	Pre_Drive_Var_Adjust_First_Drive();
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
	accele = accele_init;

	// 속도 관련 변수 초기화
	targetSpeed = targetSpeed_init;
	decele = decele_init;
	curSpeed = MIN_SPEED;

	// 좌우모터 포지션 값을 0으로 초기화
	positionVal = 0;

	// 현재 모터가 상을 잡은 횟수 초기화
	curTick_L = 0;
	curTick_R = 0;



	/*
	 * 주행문에서 쓰는 변수
	 */

	// 현재 마크 인식 상태를 직선 주행으로 초기화
	markState = MARK_STRAIGHT;

	// state machine 의 상태 업데이트
	driveState = DRIVE_STATE_IDLE;


}





