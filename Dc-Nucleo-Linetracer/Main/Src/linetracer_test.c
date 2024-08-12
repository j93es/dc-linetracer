/*
 * linetracer_test.c
 */

#include "header_init.h"




void Switch_Test() {
	Custom_OLED_Clear();

	Custom_OLED_Printf("/0 1: no");
	Custom_OLED_Printf("/1 2: no");
	Custom_OLED_Printf("/2 3: no");
	Custom_OLED_Printf("/3 1, 2: no");
	Custom_OLED_Printf("/4 1, 3: no");
	Custom_OLED_Printf("/5 2, 3: no");

	uint8_t sw = Custom_Switch_Read();
	while(sw != CUSTOM_SW_ALL){
		sw = Custom_Switch_Read();

		if (CUSTOM_SW_1 == sw){
			Custom_OLED_Printf("/0 1: yes");
		}
		if (CUSTOM_SW_2 == sw){
			Custom_OLED_Printf("/1 2: yes");
		}
		if (CUSTOM_SW_3 == sw){
			Custom_OLED_Printf("/2 3: yes");
		}
		if (CUSTOM_SW_1_2 == sw){
			Custom_OLED_Printf("/3 1, 2: yes");
		}
		if (CUSTOM_SW_1_3 == sw){
			Custom_OLED_Printf("/4 1, 3: yes");
		}
		if (CUSTOM_SW_2_3 == sw){
			Custom_OLED_Printf("/5 2, 3: yes");
		}
	}
	Custom_OLED_Clear();
}




void Sensor_Test_Raw() {
	Sensor_Start();
	Custom_OLED_Clear();

	// 센서의 Raw 값을 디스플레이에 출력해 확인하기
	while (CUSTOM_SW_3 != Custom_Switch_Read()) {
		Custom_OLED_Printf("/1%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			sensorRawVals[0], sensorRawVals[1], sensorRawVals[2], sensorRawVals[3], \
			sensorRawVals[4], sensorRawVals[5], sensorRawVals[6], sensorRawVals[7]);

		Custom_OLED_Printf("/3%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			sensorRawVals[8], sensorRawVals[9], sensorRawVals[10], sensorRawVals[11], \
			sensorRawVals[12], sensorRawVals[13], sensorRawVals[14], sensorRawVals[15]);
	}

	Custom_OLED_Clear();
	Sensor_Stop();
}








void Sensor_Test_Normalized() {
	Sensor_Start();
	Custom_OLED_Clear();

	// 센서의 Normalized 값을 디스플레이에 출력해 확인하기
	while (CUSTOM_SW_3 != Custom_Switch_Read()) {
		Custom_OLED_Printf("/1%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			sensorNormVals[0], sensorNormVals[1], sensorNormVals[2], sensorNormVals[3], \
			sensorNormVals[4], sensorNormVals[5], sensorNormVals[6], sensorNormVals[7]);

		Custom_OLED_Printf("/3%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			sensorNormVals[8], sensorNormVals[9], sensorNormVals[10], sensorNormVals[11], \
			sensorNormVals[12], sensorNormVals[13], sensorNormVals[14], sensorNormVals[15]);
	}

	Custom_OLED_Clear();
	Sensor_Stop();
}






void Sensor_Test_State() {
	uint8_t sw = 0;

	Sensor_Start();
	Custom_OLED_Clear();

	// 센서의 State 값을 디스플레이에 출력해 확인하기
	while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {
		Custom_OLED_Printf("/0threshold: %3d", threshold);

		Custom_OLED_Printf("/1%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			(irSensorState >> 15) & 1, (irSensorState >> 14) & 1, (irSensorState >> 13) & 1, (irSensorState >> 12) & 1, \
			(irSensorState >> 11) & 1, (irSensorState >> 10) & 1, (irSensorState >> 9) & 1, (irSensorState >> 8) & 1);

		Custom_OLED_Printf("/3%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w%2x/r%2x/w", \
			(irSensorState >> 7) & 1, (irSensorState >> 6) & 1, (irSensorState >> 5) & 1, (irSensorState >> 4) & 1, \
			(irSensorState >> 3) & 1, (irSensorState >> 2) & 1, (irSensorState >> 1) & 1, (irSensorState >> 0) & 1);


		if (sw == CUSTOM_SW_1) {
			if (threshold > THRESHOLD_MIN) {
				threshold -= THRESHOLD_CHANGE_VAL;
			}
		}
		else if (sw == CUSTOM_SW_2) {
			if (threshold < THRESHOLD_MAX) {
				threshold += THRESHOLD_CHANGE_VAL;
			}
		}
	}

	Custom_OLED_Clear();
	Sensor_Stop();
}


void Battery_Test_Voltage() {
	Sensor_Start();
	Custom_OLED_Clear();

	// 센서의 Normalized 값을 디스플레이에 출력해 확인하기
	while (CUSTOM_SW_3 != Custom_Switch_Read()) {

		Custom_OLED_Printf("/A%5f", sensingVoltage);
	}

	Custom_OLED_Clear();
	Sensor_Stop();
}













void MotorR_Test_Duty() {
	LL_TIM_EnableCounter(TIM11);
	LL_TIM_CC_EnableChannel(TIM11, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH2(TIM11, 0);

	LL_TIM_EnableCounter(TIM3);

	const uint16_t level_max = TIM11->ARR + 1;
	float duty_ratio = 0.0f;

	for (;;) {

		// input
		uint8_t sw = Custom_Switch_Read();

		if (sw == CUSTOM_SW_3) {
		 break;
		} else if (sw == CUSTOM_SW_1) {
		 duty_ratio -= 0.1f;
		} else if (sw == CUSTOM_SW_2) {
		 duty_ratio += 0.1f;
		}

		// get level(CCR3)
		int level = ABS(duty_ratio * level_max);

		if (level > level_max) {
		 level = level_max;
		} else if (level < 0) {
		 level = 0;
		}

		// set level(CCR3) and direction
		TIM11->CCR1 = level;
		Custom_GPIO_Set(GPIOC, 1 << 4, duty_ratio < 0 ? 1 : 0); // PC4
		Custom_GPIO_Set(GPIOC, 1 << 5, duty_ratio > 0 ? 1 : 0); // PC5
		Custom_OLED_Printf("/0Duty : %3.2f", duty_ratio);
		Custom_OLED_Printf("/1CCR3 : %4d", TIM11->CCR1);

		Custom_OLED_Printf("/2ECOD : %9d", TIM3->CNT);
	}

	TIM11->CCR1 = 0;
	LL_TIM_DisableCounter(TIM11);
	LL_TIM_CC_DisableChannel(TIM11, LL_TIM_CHANNEL_CH1);

	LL_TIM_DisableCounter(TIM3);
}


void MotorL_Test_Duty() {
	LL_TIM_EnableCounter(TIM10);
	LL_TIM_CC_EnableChannel(TIM10, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH2(TIM10, 0);

	LL_TIM_EnableCounter(TIM4);

	const uint16_t level_max = TIM10->ARR + 1;
	float duty_ratio = 0.0f;


	for (;;) {

		// input
		uint8_t sw = Custom_Switch_Read();

		if (sw == CUSTOM_SW_3) {
		 break;
		} else if (sw == CUSTOM_SW_1) {
		 duty_ratio -= 0.1f;
		} else if (sw == CUSTOM_SW_2) {
		 duty_ratio += 0.1f;
		}

		// get level(CCR3)
		int level = ABS(duty_ratio * level_max);

		if (level > level_max) {
		 level = level_max;
		} else if (level < 0) {
		 level = 0;
		}

		// set level(CCR3) and direction
		TIM10->CCR1 = level;
		Custom_GPIO_Set(GPIOB, 1 << 4, duty_ratio > 0 ? 1 : 0); // PB4
		Custom_GPIO_Set(GPIOB, 1 << 5, duty_ratio < 0 ? 1 : 0); // PB5
		Custom_OLED_Printf("/0Duty : %3.2f", duty_ratio);
		Custom_OLED_Printf("/1CCR3 : %4d", TIM10->CCR1);

		Custom_OLED_Printf("/2ECOD : %9d", TIM4->CNT);
	}

	TIM10->CCR1 = 0;
	LL_TIM_DisableCounter(TIM10);
	LL_TIM_CC_DisableChannel(TIM10, LL_TIM_CHANNEL_CH1);

	LL_TIM_DisableCounter(TIM4);
}




void MotorL_Test_PD() {

	float coefChangeVal = 5;
	float targetChangeVal = 100;

	Pre_Drive_Var_Init();

	targetSpeed = 0;

	Sensor_Start();
	Speed_Control_Start();
	MotorL_Start();

	for (;;) {

		// input
		uint8_t sw = Custom_Switch_Read();

		if (sw == CUSTOM_SW_ALL) {
		 break;
		} else if (sw == CUSTOM_SW_1) {
			pCoef -= coefChangeVal;
		} else if (sw == CUSTOM_SW_2) {
			pCoef += coefChangeVal;
		} else if (sw == CUSTOM_SW_1_2) {
			dCoef -= coefChangeVal / 1000.f;
		} else if (sw == CUSTOM_SW_2_3) {
			dCoef += coefChangeVal / 1000.f;
		} else if (sw == CUSTOM_SW_3) {
			positionCmdL += targetChangeVal * RADIAN_PER_TICK;
		}

		Custom_OLED_Printf("/0CCR    : %5d", TIM10->CCR1);
		Custom_OLED_Printf("/1curECOD: %5d", TIM4->CNT);
		Custom_OLED_Printf("/2tarECOD: %5f", positionCmdL);
		Custom_OLED_Printf("/3pCoef  : %5f", pCoef);
		Custom_OLED_Printf("/4dCoef  : %5f", dCoef);

	}

	MotorL_Stop();
	Speed_Control_Stop();
	Sensor_Stop();
}






void MotorR_Test_PD() {

	float coefChangeVal = 5;
	float targetChangeVal = 100;

	Pre_Drive_Var_Init();

	targetSpeed = 0;

	Sensor_Start();
	Speed_Control_Start();
	MotorR_Start();

	for (;;) {

		// input
		uint8_t sw = Custom_Switch_Read();

		if (sw == CUSTOM_SW_ALL) {
		 break;
		} else if (sw == CUSTOM_SW_1) {
			pCoef -= coefChangeVal;
		} else if (sw == CUSTOM_SW_2) {
			pCoef += coefChangeVal;
		} else if (sw == CUSTOM_SW_1_2) {
			dCoef -= coefChangeVal / 1000.f;
		} else if (sw == CUSTOM_SW_2_3) {
			dCoef += coefChangeVal / 1000.f;
		} else if (sw == CUSTOM_SW_3) {
			positionCmdR += targetChangeVal * RADIAN_PER_TICK;
		}

		Custom_OLED_Printf("/0CCR    : %5d", TIM11->CCR1);
		Custom_OLED_Printf("/1curECOD: %5d", TIM3->CNT);
		Custom_OLED_Printf("/2tarECOD: %5f", positionCmdR);
		Custom_OLED_Printf("/3pCoef  : %5f", pCoef);
		Custom_OLED_Printf("/4dCoef  : %5f", dCoef);

	}
	MotorR_Stop();
	Speed_Control_Stop();
	Sensor_Stop();
}





void Motor_Test_Speed() {

	Pre_Drive_Var_Init();


	// 가속도 변수 초기화
	targetAccele = 1;
	curAccele = 0;

	// 속도 관련 변수 초기화
	targetSpeed = 0;
	decele = 1;
	curSpeed = 0;


	Motor_Start();
	Sensor_Start();
	Speed_Control_Start();

	for (;;) {

		// input
		uint8_t sw = Custom_Switch_Read();

		if (sw == CUSTOM_SW_3) {
		 break;
		} else if (sw == CUSTOM_SW_1) {
			targetSpeed -= 0.1f;
		} else if (sw == CUSTOM_SW_2) {
			targetSpeed += 0.1f;
		}

		Custom_OLED_Printf("/0speed  : %3.2f", curSpeed);
		Custom_OLED_Printf("/1CCR    : %5d", TIM10->CCR1);
		Custom_OLED_Printf("/2curECOD: %5d", TIM4->CNT);
//		Custom_OLED_Printf("/3tarECOD: %5f", targetEncoderValueL_cntl)
		Custom_OLED_Printf("/5%5f", positionL );
		Custom_OLED_Printf("/5%5f", positionCmdL );

	}

	Speed_Control_Stop();
	Sensor_Stop();
	Motor_Stop();


}






//void Motor_Test_Velocity() {
//	uint8_t		sw = 0;
//	float		speed = MIN_SPEED;
//	float		maxSpeed = 2.f;
//	float		minSpeed = 1.f;
//	float		accele = 1;
//	/*
//	 * 모터 속도를 부드럽게 올렸다가 내리기를 반복한다.
//	 */
//
//	Motor_Start();
//
//	while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {
//
//		// accele / 1000인 이유는 단위 시간이 1ms이기 때문이다. 따라서 인터럽트는 500us 단위이기 때문에 인터럽트에서는 accele / 2000을 해야한다.
//		speed += accele / 1000;
//
//
//		if (speed > maxSpeed) {
//			speed = maxSpeed;
//			accele *= -1;
//		}
//		else if (speed < minSpeed) {
//			speed = minSpeed;
//			accele *= -1;
//		}
//
//		Motor_L_Speed_Control(speed);
//		Motor_R_Speed_Control(speed);
//
//		Custom_Delay_ms(1);
//	}
//	Motor_Stop();
//}














void Drive_Test_Position() {
	uint8_t	sw = 0;

	uint8_t positioningIdx = 0;

	Custom_OLED_Clear();
	Sensor_Start();
	Speed_Control_Start();

	// 좌우 모터 포지션에 관한 변수
	positionVal = 0;
	positionCoef = POSITION_COEF_INIT;

	while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

		for (int i = 0; i < 8; i++) {
			Positioning(&positioningIdx);
		}

		Custom_OLED_Printf("/0pos:     %7d", positionVal);
		Custom_OLED_Printf("/1pos:     %7d", limitedPositionVal);
	}
	Speed_Control_Stop();
	Sensor_Stop();
	Custom_OLED_Clear();
}






void Mark_Live_Test() {
	uint8_t	sw = 0;

	Sensor_Start();

    Custom_OLED_Clear();

    Pre_Drive_Var_Init();

	positionIdxMax = 9;
	positionIdxMin = 6;

    while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

    	Mark();

    	Positioning();

        switch (markStateMachine) {
        case MARK_STATE_MACHINE_IDLE:
        	Custom_OLED_Printf("/0STATE: IDLE     ");
            break;
        case MARK_STATE_MACHINE_CROSS:
        	Custom_OLED_Printf("/0STATE: CROSS    ");
            break;
        case MARK_STATE_MACHINE_MARKER:
        	Custom_OLED_Printf("/0STATE: MARK     ");
            break;
        case MARK_STATE_MACHINE_DECISION:
        	Custom_OLED_Printf("/0STATE: DECISION ");
        	break;
        default:
        	Custom_OLED_Printf("/0STATE: ------   ");
            break;
        }

        switch (markState) {
		case MARK_NONE:
			Custom_OLED_Printf("/1MARK: NONE      ");
			break;
		case MARK_STRAIGHT:
			Custom_OLED_Printf("/1MARK: STRAIGHT  ");
			break;
        case MARK_CURVE_L:
        	Custom_OLED_Printf("/1MARK: LEFT      ");
            break;
        case MARK_CURVE_R:
        	Custom_OLED_Printf("/1MARK: RIGHT     ");
            break;
        case MARK_END:
        	Custom_OLED_Printf("/1MARK: END       ");
            break;
        case MARK_CROSS:
        	Custom_OLED_Printf("/1MARK: CROSS     ");
            break;
        case MARK_LINE_OUT:
            Custom_OLED_Printf("/1MARK: LINE OUT  ");
            break;
        default:
        	Custom_OLED_Printf("/1MARK: ------    ");
        	break;
        }

    	uint16_t masking = lineMasking;

    	Custom_OLED_Printf("/2%x/r%x/w%x/r%x/w%x/r%x/w%x/r%x/w",  \
    				(masking >> 15) & 1, (masking >> 14) & 1, (masking >> 13) & 1, (masking >> 12) & 1, \
    				(masking >> 11) & 1, (masking >> 10) & 1, (masking >> 9) & 1, (masking >> 8) & 1);

    	Custom_OLED_Printf("/3%x/r%x/w%x/r%x/w%x/r%x/w%x/r%x/w", \
    				(masking >> 7) & 1, (masking >> 6) & 1, (masking >> 5) & 1, (masking >> 4) & 1, \
    				(masking >> 3) & 1, (masking >> 2) & 1, (masking >> 1) & 1, (masking >> 0) & 1);



    	masking = bothMarkMasking;

    	Custom_OLED_Printf("/4%x/r%x/w%x/r%x/w%x/r%x/w%x/r%x/w",  \
    				(masking >> 15) & 1, (masking >> 14) & 1, (masking >> 13) & 1, (masking >> 12) & 1, \
    				(masking >> 11) & 1, (masking >> 10) & 1, (masking >> 9) & 1, (masking >> 8) & 1);

    	Custom_OLED_Printf("/5%x/r%x/w%x/r%x/w%x/r%x/w%x/r%x/w", \
    				(masking >> 7) & 1, (masking >> 6) & 1, (masking >> 5) & 1, (masking >> 4) & 1, \
    				(masking >> 3) & 1, (masking >> 2) & 1, (masking >> 1) & 1, (masking >> 0) & 1);
    }

    Sensor_Stop();
}


//void Current_Setting() {
//	uint8_t		sw = 0;
//	float		acc = ACCELE_INIT;
//	float		speed = MIN_SPEED;
//	float		target = 2.0f;
//
//	Motor_Start();
//
//	while (CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {
//
//		Motor_L_Speed_Control(speed);
//		Motor_R_Speed_Control(speed);
//
//		speed += acc / 1000;
//		if (speed > target) {
//			speed = target;
//		}
//
//		Custom_Delay_ms(1);
//	}
//
//	Motor_Stop();
//}

