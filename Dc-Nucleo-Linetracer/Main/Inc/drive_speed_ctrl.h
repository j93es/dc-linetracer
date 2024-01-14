/*
 * drive_speed_ctrl.h
 */

#ifndef INC_DRIVE_SPEED_CTRL_H_
#define INC_DRIVE_SPEED_CTRL_H_





#include "drive_def_var.h"
#include "init.h"
#include "main.h"
#include "custom_delay.h"





__STATIC_INLINE void	Motor_L_Speed_Control(float speed) {
	LL_TIM_SetAutoReload(TIM3, SPEED_COEF / speed - 1);
}


__STATIC_INLINE void	Motor_R_Speed_Control(float speed) {
	LL_TIM_SetAutoReload(TIM4, SPEED_COEF / speed - 1);
}


// 가속도 및 속도 제어
__STATIC_INLINE void	Drive_Speed_Accele_Cntl() {

	if (curSpeed == targetSpeed) {

		// 속도를 targetSpeed 까지 올린 후, curAccele을 0으로 변환
		// 혹은 직선 가속 후 targetSpeed 까지 도달하지 못하고 감속한 후 감속이 종료되었으면 , curAccele을 0으로 변환
		curAccele = 0;
	}

	else if (curSpeed < targetSpeed) {

		// 가속도 제어
		curAccele += 0.01f;

		if (curAccele > targetAccele) {

			curAccele = targetAccele;
		}

		// 속도 제어
		curSpeed += curAccele / 2000;

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




//limitedPositionVal 값 업데이트
__STATIC_INLINE void	Make_Limited_Position() {

	int32_t absPositionVal = ABS(positionVal - curInlineVal);

	if (limitedPositionVal == absPositionVal) {

		return ;
	}

	// 곡선에 진입을 시작했을 때 빠르게 curve decel을 해줌
	else if (limitedPositionVal < absPositionVal) {

		limitedPositionVal += 20;
		if (limitedPositionVal > absPositionVal) {
			limitedPositionVal = absPositionVal;
		}
	}

	// 곡선에서 벗어날 때 천천히 속도를 올려줌
	else {

		limitedPositionVal -= 10;
		if (limitedPositionVal < absPositionVal) {
			limitedPositionVal = absPositionVal;
		}
	}
}


__STATIC_INLINE void	Make_Inline_Val(float finalSpeed) {

	/*
	 * l(m) 이동하는데 걸리는 시간(l(m) / 500(us)) = l(m) / curSpeed(m/s) * 2000(s/500us)
	 *
	 * l(m) 이동햇을 때 (curInlineVal == targetInlineVal)이 되도록 하는 curInlineVal의 변회량
	 * 		= targetInlineVal / { l(m) 이동하는데 걸리는 시간(l(m) / 500(us)) }
	 * 		= targetInlineVal * curSpeed / l(m) / 2000
	 */

	if (curInlineVal == targetInlineVal) {

		return ;
	}

	else if (curInlineVal < targetInlineVal) {

		curInlineVal += 20;//targetInlineVal * finalSpeed / INLINE_POSITIONING_LEN / 2000;
		if (curInlineVal > targetInlineVal) {
			curInlineVal = targetInlineVal;
		}
	}

	else {

		curInlineVal -= 20;//targetInlineVal * finalSpeed / INLINE_POSITIONING_LEN / 2000;;
		if (curInlineVal < targetInlineVal) {
			curInlineVal = targetInlineVal;
		}
	}
}



// 500us마다 호출됨.
__STATIC_INLINE void	Drive_TIM9_IRQ() {

	float	finalSpeed;


	// 가속도 및 속도 제어
	Drive_Speed_Accele_Cntl();

	// limitedPositionVal 값 업데이트
	Make_Limited_Position();

	// 포지션 값에 따른 감속
	finalSpeed = curSpeed * curveDeceleCoef / (limitedPositionVal + curveDeceleCoef);

	// inLine 값 생성
	Make_Inline_Val(finalSpeed);

	//position 값에 따른 좌우 모터 속도 조정
	Motor_L_Speed_Control( finalSpeed * (1 + (positionVal - curInlineVal) * positionCoef) );
	Motor_R_Speed_Control( finalSpeed * (1 - (positionVal - curInlineVal) * positionCoef) );
}





// 피트인 함수
__STATIC_INLINE void	Drive_Fit_In(float s, float pinSpeed) {

	targetSpeed = pinSpeed;
	decele = ABS( (pinSpeed - curSpeed) * (pinSpeed + curSpeed) ) / (2 * s);
}





__STATIC_INLINE uint8_t	Is_Drive_End(uint8_t exitEcho) {

	// endMark || lineOut
	if (endMarkCnt >= 2 || markState == MARK_LINE_OUT) {

		Drive_Fit_In(pitInLen, PIT_IN_TARGET_SPEED);

		while (curSpeed > DRIVE_END_DELAY_SPEED) {
			//Drive_Speed_Cntl();
		}

		Custom_Delay_ms(DRIVE_END_DELAY_TIME_MS);

		if (endMarkCnt >= 2) {

			exitEcho = EXIT_ECHO_END_MARK;

			optimizeLevel += 1;
		}
		else {

			exitEcho = EXIT_ECHO_LINE_OUT;
		}
	}

	return exitEcho;
}





#endif /* INC_DRIVE_SPEED_CTRL_H_ */
