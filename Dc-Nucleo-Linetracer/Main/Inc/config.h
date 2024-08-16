/*
 * drive_def_var.h
 */


#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_


#include <stdint.h>
#include <stdbool.h>



// 공용 매크로
#define CUSTOM_FALSE				0
#define CUSTOM_TRUE					1
#define ABS(x) 						( ((x) < 0) ? (-1 * (x)) : (x) )
#define GET_MAX(x, y)				( ((x) > (y)) ? (x) : (y) )
#define GET_MIN(x, y)				( ((x) < (y)) ? (x) : (y) )
#define INT_SWAP(a, b)				{ a ^= b; b ^= a; a ^= b; }
#define FLOAT_SWAP(a, b)			{ float _tmp = a; a = b; b = _tmp; }



// hw spec
#define TIRE_RADIUS					0.036f
#define ROBOT_WIDTH					0.2f


// pd 제어 매크로
#define P_COEF_INIT					160.f
#define D_COEF_INIT					0.256f
//#define T_ENCODER_MAX				65536
#define MOTOR_RESISTANCE			7.14f
#define MOTOR_KE					0.0330f
#define TICK_PER_M					( ENCODER_VALUE_PER_CIRCLE / (TIRE_RADIUS * 3.141592f * MOTOR_GEAR_RATIO) )
#define RADIAN_PER_M				(1 / (3.141592 * TIRE_RADIUS * MOTOR_GEAR_RATIO))
#define RADIAN_PER_TICK				( 1 / ENCODER_VALUE_PER_CIRCLE )


// 속도와 관련된 매크로
#define MIN_SPEED					0.0f

#define JERK_COEF					0.2f

#define ACCELE_INIT					8.0f
#define DECELE_INIT					7.0f

#define TARGET_SPEED_INIT			2.8f
#define STRAIGHT_BOOST_SPEED_INIT	7.f
#define CURVE_BOOST_SPEED_INIT		4.f


// 커브에서 어느 정도 감속할지 결정하는 매크로
#define CURVE_DECELE_COEF_INIT		24500.f


// POSITION_COEF(포지션 상수)를 도출하기 위한 매크로				// m
#define POSITION_COEF_INIT			0.00006f


// motor
#define MOTOR_CONTROL_INTERVAL_S	0.0005f
#define ENCODER_VALUE_PER_CIRCLE	2048.f
#define MOTOR_GEAR_RATIO			( 17.f / 69.f )



// 1차 주행인지 2차주행 판단 매크로
#define FIRST_DRIVE					0
#define SECOND_DRIVE				1


// state machine 비트 마스킹
#define LINE_MASKING_INIT			0b0000011111100000
#define RIGHT_MARK_MASKING_INIT		0b0000000000001110
#define LEFT_MARK_MASKING_INIT		0b0111000000000000
#define ALL_MARK_MASKING			0b1111111111111111



// state machine에서 나온 상태
#define MARK_STATE_MACHINE_IDLE			0
#define MARK_STATE_MACHINE_CROSS		1
#define MARK_STATE_MACHINE_MARKER		2
#define MARK_STATE_MACHINE_DECISION		3
#define MARK_STATE_MACHINE_LINE_OUT		4


// 현재 mark의 상태값 매크로
#define MARK_NONE					0
#define MARK_STRAIGHT				1
#define MARK_CURVE_R				2
#define MARK_CURVE_L				3
#define MARK_END					4
#define MARK_CROSS					5
#define MARK_LINE_OUT				6

#define CURVATURE_0					0
#define CURVATURE_45				1
#define CURVATURE_90				2
#define CURVATURE_135				3
#define CURVATURE_180				4
#define CURVATURE_235				5
#define CURVATURE_270				6


// 부스트 컨트롤 매크로
#define BOOST_CNTL_IDLE				0
#define BOOST_CNTL_ACCELE			1
#define BOOST_CNTL_DECELE			2
#define BOOST_CNTL_END				3


// 인라인 컨트롤 매크로
#define INLINE_CNTL_IDLE			0
#define INLINE_CNTL_CURVE_IN		1
#define INLINE_CNTL_CURVE_OUT		2
#define INLINE_CNTL_END				3



// exitEcho 관련 매크로
#define EXIT_ECHO_IDLE				0
#define EXIT_ECHO_END_MARK			1
#define EXIT_ECHO_LINE_OUT			2


// 라인 아웃 일 때 몇 초 딜레이 할지
#define LINE_OUT_DELAY_MS			200



// 피트인 관련 매크로
#define PIT_IN_LEN_INIT				0.15f
#define PIT_IN_TARGET_SPEED			MIN_SPEED


// 주행이 종료되었을 때 모터 종료 딜레이
#define DRIVE_END_DELAY_SPEED		0.2f
#define DRIVE_END_DELAY_TIME_MS		1000


// 2차 주행에서 어느 정도 지나면 가감속할 지 결정하는 매크로

//// 직선에 진입한 후 어느정도 이동한 후 가속할지
#define ACCELE_START_TICK_INIT		( 0.1f * TICK_PER_M )

// 감속 안전거리 (최소 20cm 이상)
#define DECELE_END_TICK_INIT		( 0.2f * TICK_PER_M )

// 감속 안전비율
#define DECELE_END_RATIO_INIT		0.3f

#define MIN_STRAIGHT_BOOST_TICK		( 0.1f * TICK_PER_M )

#define MIN_CURVE_BOOST_TICK		( 1.2f * TICK_PER_M )


// 인라인 주행 관련 매크로
#define ABS_INLINE_TARGET_POSITION			8000
#define INLINE_POSITIONING_TICK				( 0.13f * TICK_PER_M )



// 1차주행, 2차 주행의 driveData 관련 매크로
#define STOP_END_MARK_CNT_INIT		2
#define MAX_DRIVE_DATA_LEN			320
#define T_DRIVE_DATA_INIT			{ 0, 0, MARK_NONE, 0, CURVATURE_0 }


// 최대 크로스 개수
#define MAX_CROSS_CNT				128
#define LAST_STRAIGHT_TARGET_SPEED	2.6f


// 마크 샘플링
#define MARK_SAMPLING_MAX_LEN			10
#define MARK_SAMPLING_METER				0.004
#define MARK_SAMPLING_TICK				( MARK_SAMPLING_METER * TICK_PER_M )
// (MARK_SAMPLING_METER * MARK_SAMPLING_MAX_LEN == ncm) 간격으로 accum 진행





// 센서

#define IR_SENSOR_MID			7

#define IR_SENSOR_LEN			16
#define IR_SENSOR_LEN_HALF		8

#define WINDOW_SIZE_HALF		2

#define	THRESHOLD_MAX 			245
#define	THRESHOLD_MIN			10
#define	THRESHOLD_CHANGE_VAL	5
#define	THRESHOLD_INIT			140



#define STRAIGHT_ADJUST_TICK	( 2.f * TICK_PER_M )



typedef int16_t		t_encoder;
typedef int32_t			t_tick;





// 1차주행, 2차 주행의 driveData 구조체
typedef struct	s_driveData {

		// 현재 마크에서 이동한 tick(거리)
		t_tick	tickCnt_L;
		t_tick	tickCnt_R;

		// 현재 마크의 상태
		uint8_t		markState;

		// 마크 종료시점에서의 크로스 개수
		uint8_t		crossCnt;

		uint16_t		curvature;

}				t_driveData;







typedef struct s_mark_masking {
    volatile uint16_t left_mask[IR_SENSOR_LEN];
    volatile uint16_t right_mask[IR_SENSOR_LEN];
    volatile uint16_t line_mask[IR_SENSOR_LEN];
}				t_mark_masking;





/*
 * 인터럽트에서 쓰는 변수
 */


// pd 제어에 사용하는 변수
extern volatile uint32_t 	levelMaxCCR;
extern volatile float		positionCmdL;
extern volatile float		positionL;
extern volatile float		positionCmdR;
extern volatile float		positionR;
//extern volatile int32_t		prevErrorL;
//extern volatile int32_t		prevErrorR;
//extern volatile t_encoder	targetEncoderValueL_cntl;
//extern volatile t_encoder	targetEncoderValueR_cntl;
extern volatile t_encoder	prevEncoderValueL;
extern volatile t_encoder	prevEncoderValueR;
extern volatile float		pCoef;
extern volatile float		dCoef;


// 초기의 속도 값에 관한 변수
extern volatile float		targetSpeed_init;
extern volatile float		targetAccele_init;
extern volatile float		decele_init;


// 좌우 모터 포지션에 관한 변수
extern volatile int32_t		positionVal;
extern volatile float		positionCoef;
extern volatile int32_t		limitedPositionVal;
extern volatile int32_t		prevPositionValCmd;

extern volatile uint8_t		positionIdxMax;
extern volatile uint8_t		positionIdxMin;
extern volatile int32_t		positionSum;
extern volatile int32_t		sensorNormValsSum;

extern volatile int8_t		curPositionIrSensorMid;


// 주행 중 변하는 속도 값에 관한 변수
extern volatile float		targetAccele;
extern volatile float		curAccele;
extern volatile float		decele;

extern volatile float		targetSpeed;
extern volatile float		curSpeed;
extern volatile float		starightBoostSpeed;
extern volatile float		curveBoostSpeed;

extern volatile float		curveDeceleCoef;


// 현재 모터에 몇번 상이 잡혔는지를 카운트하는 변수
extern volatile t_tick		curTick_L;
extern volatile t_tick		curTick_R;
extern volatile float		curMarkSamplingTick;


// 2차 주행 inline
extern volatile int32_t		targetInlineVal;
extern volatile int32_t		curInlineVal;





/*
 * 주행문에서 쓰는 변수
 */


// 현재 직진인지 커브인지 등을 저장하는 변수
extern uint8_t				markState;



// state machine 비트 마스킹
extern uint16_t				lineMasking;
extern uint16_t				rightMarkMasking;
extern uint16_t				leftMarkMasking;
extern uint16_t				bothMarkMasking;
extern uint16_t				markAreaMasking;

extern t_mark_masking		markMasking;



// state machine 의 상태
extern uint8_t				markStateMachine;


// 주행 컨트롤 변수
extern uint8_t				starightBoostCntl;
extern uint8_t				curveBoostCntl;
extern uint8_t				curveInlineCntl;


// 직선 주행, 곡선 인라인 최적화 레벨
extern uint8_t				isStraightBoostEnabled;
extern uint8_t				isCurveBoostEnabled;
extern uint8_t				isInlineDriveEnabled;


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


//end mark를 몇 번 봤는지 카운트하는 변수
extern uint8_t				endMarkCnt;
extern uint8_t 				stopEndMarkCnt;

// 피트인 거리
extern float				pitInLen;


// state machine 에서 사용
//센서 값 누적
extern uint8_t				irSensorStateSum;


// 2차 주행 직선가속에서 사용
extern float				acceleStartTick;
extern float				deceleEndTick;
extern float				deceleEndRatio;

extern float				isLastStraight;


extern float				targetMarkSamplingTick;
extern uint8_t				markSampling[MARK_SAMPLING_MAX_LEN];




#endif //INC_CONFIG_H_
