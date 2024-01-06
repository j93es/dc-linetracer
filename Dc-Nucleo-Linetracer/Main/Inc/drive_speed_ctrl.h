/*
 * drive_speed_ctrl.h
 */

#ifndef INC_DRIVE_SPEED_CTRL_H_
#define INC_DRIVE_SPEED_CTRL_H_





#include "init.h"
#include "main.h"
#include "sensor.h"
#include "custom_delay.h"




// 속도와 관련된 매크로
#define MIN_SPEED					0.01f

#define ACCELE_INIT					3.0f
#define DECELE_INIT					6.0f

#define TARGET_SPEED_INIT			1.7f


// 커브에서 어느 정도 감속할지 결정하는 매크로
#define CURVE_DECELE_COEF_INIT		22500.f


// POSITION_COEF(포지션 상수)를 도출하기 위한 매크로
#define TIRE_RADIUS					0.026f					// m
#define POSITION_COEF_INIT			0.00006f

/*
 * (2 * l(m) * 3.14159) / (t(s) * 200) = v(m/s) * (arr+1)
 *
 * t =  1 / 1Mhz = 1 / 1,000,000 = 타이머 주기
 * 1 / (t * 200) = 5,000
 *
 * l(m) = 타이어 반지름
 * 2 * l * 3.14159
 *
 * v * (arr + 1) = SPEED_COEF
 */
//#define SPEED_COEF					( 31415.92f * TIRE_RADIUS )

/*
 * (2 * l(m) * 3.14159) / (t(s) * 400) = v(m/s) * (arr+1)
 *
 * t =  1 / 1Mhz = 1 / 1,000,000 = 타이머 주기
 * 1 / (t * 400) = 2,500
 *
 * l(m) = 타이어 반지름
 * 2 * l * 3.14159
 *
 * v * (arr + 1) = SPEED_COEF
 */
#define SPEED_COEF					( 15707.96f * TIRE_RADIUS )


// 1 m 당 tick 개수
/*
 * 200(바퀴가 1바퀴 도는데 소요되는 tick 개수) * { 1(m) / (2 * TIRE_RADIUS * 3.14159) }(1바퀴의 거리) == { 1m 가는데 소요되는 바퀴 회전 횟수 }
 */
//#define TICK_PER_M					( 31.831f / TIRE_RADIUS )

/*
 * 400(바퀴가 1바퀴 도는데 소요되는 tick 개수) * { 1(m) / (2 * TIRE_RADIUS * 3.14159) }(1바퀴의 거리) == { 1m 가는데 소요되는 바퀴 회전 횟수 }
 */
#define TICK_PER_M					( 63.662f / TIRE_RADIUS )




// 초기의 속도 값에 관한 변수
extern volatile float		targetSpeed_init;
extern volatile float		accele_init;
extern volatile float		decele_init;


// 좌우 모터 포지션에 관한 변수
extern volatile int32_t		positionVal;
extern volatile float		positionCoef;

extern volatile int32_t		positionSum;
extern volatile int32_t		sensorNormValsSum;

extern volatile int32_t		positionTable[8];


// 주행 중 변하는 속도 값에 관한 변수
extern volatile float		accele;
extern volatile float		decele;

extern volatile float		targetSpeed;
extern volatile float		curSpeed;

extern volatile float		curveDeceleCoef;




void	Make_Position_Val();
void	Drive_Speed_Accele_Cntl();



__STATIC_INLINE void	Motor_L_Speed_Control(float speed) {
	LL_TIM_SetAutoReload(TIM3, SPEED_COEF / speed - 1);
}


__STATIC_INLINE void	Motor_R_Speed_Control(float speed) {
	LL_TIM_SetAutoReload(TIM4, SPEED_COEF / speed - 1);
}




// 500us마다 호출됨.
__STATIC_INLINE void	Drive_TIM9_IRQ() {

	float	finalSpeed;


	// 가속도 및 속도 제어
	Drive_Speed_Accele_Cntl();

	Make_Position_Val();

	// 포지션 값에 따른 감속
	finalSpeed = curSpeed * curveDeceleCoef / (curveDeceleCoef + ABS(positionVal));

	//position 값에 따른 좌우 모터 속도 조정
	Motor_L_Speed_Control( finalSpeed * (1 + positionVal * positionCoef) );
	Motor_R_Speed_Control( finalSpeed * (1 - positionVal * positionCoef) );
}





#endif /* INC_DRIVE_SPEED_CTRL_H_ */
