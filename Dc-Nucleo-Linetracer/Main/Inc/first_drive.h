/*
 * first_drive.h
 */

#ifndef INC_FIRST_DRIVE_H_
#define INC_FIRST_DRIVE_H_


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



// exitEcho 관련 매크로
#define EXIT_ECHO_IDLE				0
#define EXIT_ECHO_END_MARK			1
#define EXIT_ECHO_LINE_OUT			2


// 라인 아웃 일 때 몇 초 딜레이 할지
#define LINE_OUT_DELAY_MS			0



// 피트인 관련 매크로
#define PIT_IN_LEN_INIT				0.18f
#define PIT_IN_TARGET_SPEED			MIN_SPEED


// 주행이 종료되었을 때 모터 종료 딜레이
#define DRIVE_END_DELAY_SPEED		0.1f
#define DRIVE_END_DELAY_TIME_MS		200



// state machine 의 상태
extern uint8_t				driveState;
extern uint8_t				markState;


//end mark를 몇 번 봤는지 카운트하는 변수
extern uint8_t				endMarkCnt;


// 피트인 거리
extern float				pitInLen;


// state machine 에서 사용
//센서 값 누적
extern uint8_t				sensorStateSum;





void	First_Drive();
void	Drive_State_Machine();
void	Decision(uint8_t sensorStateSum);
void	Drive_Fit_In(float s, float pinSpeed);
uint8_t	Is_Drive_End(uint8_t exitEcho);


#endif
