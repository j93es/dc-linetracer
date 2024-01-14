/*
 * drive_def_var.h
 */


#ifndef INC_DRIVE_DEF_VAR_H_
#define INC_DRIVE_DEF_VAR_H_


#include <stdint.h>
#include <stdbool.h>


// 공용 매크로
#define CUSTOM_FALSE				0
#define CUSTOM_TRUE					1
#define ABS(x) 						( ((x) < 0) ? (-1 * (x)) : (x) )
#define GET_MIN(x, y)				( ((x) < (y)) ? (x) : (y) )


// 속도와 관련된 매크로
#define MIN_SPEED					0.01f

#define ACCELE_INIT					7.0f
#define DECELE_INIT					6.0f

#define TARGET_SPEED_INIT			2.45f
#define BOOST_SPEED_INIT			4.5f


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


// 1차 주행인지 2차주행 판단 매크로
#define FIRST_DRIVE					0
#define SECOND_DRIVE				1


// state machine에서 나온 상태
#define DRIVE_STATE_IDLE			0
#define DRIVE_STATE_CROSS			1
#define DRIVE_STATE_MARKER			2
#define DRIVE_STATE_DECISION		3
#define DRIVE_DECISION_LINE_OUT		4


// 현재 mark의 상태값 매크로
#define MARK_NONE					0
#define MARK_STRAIGHT				1
#define MARK_CURVE_R				2
#define MARK_CURVE_L				3
#define MARK_END					4
#define MARK_CROSS					5
#define MARK_LINE_OUT				6


// 부스트 컨트롤 매크로
#define BOOST_CNTL_IDLE				0
#define BOOST_CNTL_ACCELE			1
#define BOOST_CNTL_DECELE			2
#define BOOST_CNTL_END				3


// 부스트 컨트롤 매크로
#define INLINE_CNTL_IDLE			0
#define INLINE_CNTL_CURVE_IN		1
#define INLINE_CNTL_CURVE_OUT		2
#define INLINE_CNTL_END				3


// 주행 최적화 레벨 조정
#define OPTIMIZE_LEVEL_NONE			0
#define OPTIMIZE_LEVEL_STRAIGHT		1
#define OPTIMIZE_LEVEL_CURVE		2


// exitEcho 관련 매크로
#define EXIT_ECHO_IDLE				0
#define EXIT_ECHO_END_MARK			1
#define EXIT_ECHO_LINE_OUT			2


// 라인 아웃 일 때 몇 초 딜레이 할지
#define LINE_OUT_DELAY_MS			200



// 피트인 관련 매크로
#define PIT_IN_LEN_INIT				0.18f
#define PIT_IN_TARGET_SPEED			MIN_SPEED


// 주행이 종료되었을 때 모터 종료 딜레이
#define DRIVE_END_DELAY_SPEED		0.1f
#define DRIVE_END_DELAY_TIME_MS		200


// 2차 주행에서 어느 정도 지나면 가감속할 지 결정하는 매크로

// 직선에 진입한 후 어느정도 이동한 후 가속할지
#define ACCELE_START_TICK_INIT		( 0.1f * TICK_PER_M )

// 감속 안전거리 (최소 20cm 이상)
#define DECELE_END_TICK_INIT		( 0.2f * TICK_PER_M )

// 감속 안전비율
#define DECELE_END_RATIO_INIT		0.2f


// 인라인 주행 관련 매크로
#define ABS_INLINE_TARGET_POSITION	4000
#define INLINE_END_RATIO			0.2f
//#define INLINE_POSITIONING_LEN		0.05f



// 1차주행, 2차 주행의 driveData 관련 매크로
#define MAX_DRIVE_DATA_LEN			512
#define T_DRIVE_DATA_INIT			{ 0, 0, MARK_NONE, 0 }


// 최대 크로스 개수
#define MAX_CROSS_CNT				128






// 1차주행, 2차 주행의 driveData 구조체
typedef struct	s_driveData {

		// 현재 마크에서 이동한 tick(거리)
		uint16_t	tickCnt_L;
		uint16_t	tickCnt_R;

		// 현재 마크의 상태
		uint8_t		markState;

		// 마크 종료시점에서의 크로스 개수
		uint8_t		crossCnt;

}				t_driveData;





/*
 * 인터럽트에서 쓰는 변수
 */


// 초기의 속도 값에 관한 변수
extern volatile float		targetSpeed_init;
extern volatile float		targetAccele_init;
extern volatile float		decele_init;


// 좌우 모터 포지션에 관한 변수
extern volatile int32_t		positionVal;
extern volatile float		positionCoef;
extern volatile int32_t		limitedPositionVal;

extern volatile uint8_t		positionIdxMax;
extern volatile uint8_t		positionIdxMin;
extern volatile int32_t		positionSum;
extern volatile int32_t		sensorNormValsSum;


// 주행 중 변하는 속도 값에 관한 변수
extern volatile float		targetAccele;
extern volatile float		curAccele;
extern volatile float		decele;

extern volatile float		targetSpeed;
extern volatile float		curSpeed;
extern volatile float		boostSpeed;

extern volatile float		curveDeceleCoef;


// 현재 모터에 몇번 상이 잡혔는지를 카운트하는 변수
extern volatile uint32_t	curTick_L;
extern volatile uint32_t	curTick_R;


// 2차 주행 inline
extern volatile int32_t		targetInlineVal ;
extern volatile int32_t		curInlineVal;





/*
 * 주행문에서 쓰는 변수
 */


// 현재 직진인지 커브인지 등을 저장하는 변수
extern uint8_t				markState;


// state machone 의 상태
extern uint8_t				driveState;


// 주행 컨트롤 변수
extern uint8_t				starightBoostCntl;
extern uint8_t				curveInlineCntl;


// 직선 주행, 곡선 인라인 최적화 레벨
extern uint8_t				optimizeLevel;


// 2차주행에서 마크를 정확히 읽었는지 판단
extern uint8_t				isReadAllMark;


// driveData를 저장하고 접근하게 해주는 변수들
extern t_driveData			driveData[MAX_DRIVE_DATA_LEN];


// 1차 주행 데이터 임시저장
extern t_driveData			driveDataBuffer[MAX_DRIVE_DATA_LEN];


// driveData 인덱스
extern uint16_t				driveDataIdx;


// 2차 주행에서 사용하는 cross 테이블
/*
 *    n번째 크로스(crossCnt)		0		1		...		50
 *    m번째 마크(driveDataIdx)		4(3)	6(5)	...		98
 *
 *    (0번째 마크에서 크로스를 읽었을 때 1번째 마크로 저장되도록 함, 0은 값이 없는 상태를 나타냄)
 */
extern uint16_t				crossCntTable[MAX_CROSS_CNT];


// 1차 주행에서 cross 테이블 임시 저장
extern uint16_t				crossCntTableBuffer[MAX_CROSS_CNT];


// 현재까지 읽은 크로스 개수
extern uint16_t				crossCnt;


// 현재 마크가 시작된 tick
extern uint32_t				markStartTick_L;
extern uint32_t				markStartTick_R;


//end mark를 몇 번 봤는지 카운트하는 변수
extern uint8_t				endMarkCnt;


// 피트인 거리
extern float				pitInLen;


// state machine 에서 사용
//센서 값 누적
extern uint8_t				sensorStateSum;


// 2차 주행 직선가속에서 사용
extern float				acceleStartTick;
extern float				deceleEndTick;
extern float				deceleEndRatio;




#endif //INC_DRIVE_DEF_VAR_H_
