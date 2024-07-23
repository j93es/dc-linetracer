/*
 * drive_def_var.c
 */

#include "header_init.h"



/*
 * 인터럽트에서 쓰는 변수
 */


// pd 제어에 사용하는 변수
volatile uint32_t		levelMaxCCR;

volatile int32_t		prevErrorL = 0;
volatile int32_t		prevErrorR = 0;
volatile t_encoder		targetEncoderValueL_cntl = 0;
volatile t_encoder		targetEncoderValueR_cntl = 0;
volatile t_encoder		prevCurEncoderValueL = 0;
volatile t_encoder		prevCurEncoderValueR = 0;
volatile float			pCoef = P_COEF_INIT;
volatile float			dCoef = D_COEF_INIT;



// 초기의 속도 값에 관한 변수
volatile float			targetSpeed_init = TARGET_SPEED_INIT;
volatile float			targetAccele_init = ACCELE_INIT;
volatile float			decele_init = DECELE_INIT;


// 좌우 모터 포지션에 관한 변수
volatile int32_t		positionVal = 0;
volatile float			positionCoef = POSITION_COEF_INIT;
volatile int32_t		limitedPositionVal = 0;


// positionVal을 windowing하여 구하는 것에 사용되는 변수 초기화
volatile uint8_t		positionIdxMax = 9;
volatile uint8_t		positionIdxMin = 6;
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
volatile t_tick			curTick_L = 0;
volatile t_tick			curTick_R = 0;



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


// mark masking
// state machine 비트 마스킹
uint16_t				lineMasking = LINE_MASKING_INIT;
uint16_t				rightMarkMasking = RIGHT_MARK_MASKING_INIT;
uint16_t				leftMarkMasking = LEFT_MARK_MASKING_INIT;
uint16_t				bothMarkMasking = RIGHT_MARK_MASKING_INIT | LEFT_MARK_MASKING_INIT;
uint16_t				markAreaMasking =  ~(LINE_MASKING_INIT << 1 | LINE_MASKING_INIT >> 1);;


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


//end mark를 몇 번 봤는지 카운트하는 변수
uint8_t					endMarkCnt = 0;


// 피트인 거리
float					pitInLen = PIT_IN_LEN_INIT;


// state machine 에서 사용
//센서 값 누적
uint8_t					irSensorStateSum = 0x00;


// 2차 주행 직선가속에서 사용
float					acceleStartTick = ACCELE_START_TICK_INIT;

// 안전 거리
float					deceleEndTick = DECELE_END_TICK_INIT;

// 안전 비율
float					deceleEndRatio = DECELE_END_RATIO_INIT;
