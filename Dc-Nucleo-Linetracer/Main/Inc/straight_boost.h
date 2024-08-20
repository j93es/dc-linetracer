
#ifndef INC_STRAIGTH_BOOST_H_
#define INC_STRAIGTH_BOOST_H_


#include <config.h>
#include "main.h"


__STATIC_INLINE void Straight_Boost() {

	// 감속 상수
	static float	deceleEndTickCoef;
	static float	deceleEndTickCoef_L;
	static float	deceleEndTickCoef_R;
	static float	finalDeceleEndTick_L;
	static float	finalDeceleEndTick_R;


	// 직선 가속
	switch (starightBoostCntl) {

			// 초기 상태
			case BOOST_CNTL_IDLE :


					// 최적화 레벨이 직선 가속 이상 일 때
					if (isStraightBoostEnabled) {


						finalDeceleEndTick_L =	driveData[driveDataIdx].tickCnt_L \
												- GET_MIN(deceleEndTick, deceleEndRatio * driveData[driveDataIdx].tickCnt_L);

						finalDeceleEndTick_R =	driveData[driveDataIdx].tickCnt_R \
												- GET_MIN(deceleEndTick, deceleEndRatio * driveData[driveDataIdx].tickCnt_R);

						// 최소 부스트 거리 이상이고 이미 가속을 하고 다시 IDLE 상태에 도달하지 않았을때
						if (curTick_L < driveData[driveDataIdx].tickCnt_L - MIN_STRAIGHT_BOOST_TICK \
							&& curTick_R < driveData[driveDataIdx].tickCnt_R - MIN_STRAIGHT_BOOST_TICK)  {

							// deceleEndTickCoef 업데이트
							deceleEndTickCoef = 2 * decele / TICK_PER_M;

							deceleEndTickCoef_L =	deceleEndTickCoef * finalDeceleEndTick_L \
													+ targetSpeed_init * targetSpeed_init;

							deceleEndTickCoef_R =	deceleEndTickCoef * finalDeceleEndTick_R \
													+ targetSpeed_init * targetSpeed_init;

							if (isLastStraight) {

								deceleEndTickCoef_L =	deceleEndTickCoef * finalDeceleEndTick_L \
														+ LAST_STRAIGHT_TARGET_SPEED * LAST_STRAIGHT_TARGET_SPEED;

								deceleEndTickCoef_R =	deceleEndTickCoef * finalDeceleEndTick_R \
														+ LAST_STRAIGHT_TARGET_SPEED * LAST_STRAIGHT_TARGET_SPEED;
							}

							starightBoostCntl = BOOST_CNTL_ACCELE;
						}
					}

					break ;


			// 부스트 가속 컨드롤
			case BOOST_CNTL_ACCELE :

					// 직선 구간 진입 후 ACCELE_START_TICK만큼 지났을 때 부스트
					if (curTick_L > acceleStartTick \
					 && curTick_R > acceleStartTick) {

						// starightBoostSpeed로 가속
						targetSpeed = starightBoostSpeed;

						starightBoostCntl = BOOST_CNTL_DECELE;
					}

					break;


			// 부스트 감속 컨트롤
			case BOOST_CNTL_DECELE :

					// decel이 시작되었을 경우
					/*
					 * fit_in 함수를 역산
					 * 현재 감속도를 토대로 감속하는데에 필요한 감속거리를 산출
					 * 산출된 감속거리와 실제 남은 거리를 비교
					 * 밑은 원본 식
					 *
					 * ( driveData[driveDataIdx].tickCnt - (curTick - markStartTick) - deceleEndTick ) / TICK_PER_M (실제 남은 거리)
					 * 	 <= (curSpeed * curSpeed - targetSpeed_init * targetSpeed_init) / (2 * decele) (감속도를 토대로 감속하는데에 필요한 감속거리)
					 *
					 * 위의 원본 식에서 고정된 값을 상수로 만든 뒤, 나눗셈을 없애면 식이 밑의 식이 도출됨
					 */

					if (deceleEndTickCoef_L < curSpeed * curSpeed + curTick_L * deceleEndTickCoef \
					 || deceleEndTickCoef_R < curSpeed * curSpeed + curTick_R * deceleEndTickCoef) {

						if (!isLastStraight) {
						// targetSpeed_init로 감속
							targetSpeed = targetSpeed_init;
						} else {
							targetSpeed = LAST_STRAIGHT_TARGET_SPEED;
						}


						starightBoostCntl = BOOST_CNTL_END;
					}

					break ;



			// 부스트가 종료되었을 때
			case BOOST_CNTL_END :

					// 직선이 다시 가속 안할 정도로 남았을 경우
					if (curTick_L > driveData[driveDataIdx].tickCnt_L - MIN_STRAIGHT_BOOST_TICK \
					 || curTick_R > driveData[driveDataIdx].tickCnt_R - MIN_STRAIGHT_BOOST_TICK) {

						// 부스트 중 차체가 떳을 때 크로스를 못읽는 경우를 방지함
						crossCnt = driveData[driveDataIdx].crossCnt;

						// 크로스에서 이빨 빠지는거 방지
						if (markStateMachine == MARK_STATE_MACHINE_CROSS) {
							markStateMachine = MARK_STATE_MACHINE_IDLE;
						}

						starightBoostCntl = BOOST_CNTL_IDLE;
					}

					break ;
	}
}


#endif
