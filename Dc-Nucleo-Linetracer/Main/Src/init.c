/*
 * init.c
 */

#include "header_init.h"
#include "core_cm4.h"


/*
 * syscalls.c에 있는 _io_putchar 재정의
 */
int __io_putchar(int ch)
{
	while (!LL_USART_IsActiveFlag_TXE(USART2));
	LL_USART_TransmitData8(USART2, (char)ch);
	return ch;
}



void Init() {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	/*
	 * STM 보드와 컴퓨터 간 UART 통신을 통해 컴퓨터 터미널로 디버깅할 수 있도록 USART2를 활성화한다.
	 */
	LL_USART_Enable(USART2);

	/*
	* 1ms 주기로 동작하는 SysTick을 기화한다. custom_delay.h 파일의 Custom_Delay_Get_SysTick
	* 현재 SvsTick이 초기화된 시점부터 경과한 시간을 구할 수 있다.
	*/
	Custom_Delay_Init_SysTick();

	/*
	 * OLED를 사용하기 전에는 Custom_OLED_Init 함수를 호출하여 여러가지 초기화를 수행해야 한다.
	 * 이 함수는 OLED 처음 쓰기 전에 딱 한 번만 호출하면 된다.
	 */
	Custom_OLED_Init();

	/*
	 * 플래시를 사용하기 전에는 Custom_FileSystem_Load 함수를 호출하여 플래시 정보를 불러와야 한다.
	 * 이 함수는 플래시를 처음 쓰기 전에 딱 한 번만 호출하면 된다.
	 */
	Custom_FileSystem_Load();

	/**
	 * Custom_OLED_Printf 함수는 C언어에서 printf와 동일하게 동작한다.
	 * 즉, %d, %f 등의 서식 문자를 사용하여 숫자를 출력할 수 있다.
	 * 다만 특수한 기능들 추가되어있는데, /0이라는 부분이 있으면 첫 번째 줄의 첫 번째 칸으로 돌아가고,
	 * /1이라는 부분이 있으면 두 번째 줄의 첫 번째 칸으로 돌아간다.
	 * 그리고 /r, /g, /b라는 부분이 있으면 각각 문자를 빨강, 초록, 파랑으로 출력한다.
	 * 즉, 아래 예제에서는 첫 번째 줄에 "Hello"를 출력한 후, 두 번째 줄의 첫 번째 칸으로 커서가 이동하고 파란색 글씨로 "ZETIN!"을 쓴다.
	 */
	Custom_OLED_Init();
	Custom_OLED_Printf("/0Hello, /1/bZETIN!");
	Custom_Delay_ms(1000);

	/*
	 * 아래는 스위치를 사용하는 예제다.
	 * Custom_Switch_Read 함수 내부에는 1ms 딜레이가 존재하기 때문에, 이 함수를 주행 알고리즘 내부에 집어넣으면 성능이 크게 떨어지니 주의한.
	 */
	t_menuData menus[] = {
			{ "Calibration    ", Sensor_Calibration },
			{ "First Drive    ", First_Drive },
			{ "Second Drive   ", Second_Drive },
			{ "Test Raw       ", Sensor_Test_Raw },
			{ "Test Normalized", Sensor_Test_Normalized },
			{ "Test State     ", Sensor_Test_State },
			{ "Test Voltage   ", Battery_Test_Voltage },
			{ "Test Position  ", Drive_Test_Position },
			{ "Test L PD      ", MotorL_Test_PD },
			{ "Test R PD      ", MotorR_Test_PD },
			{ "Test Speed     ", Motor_Test_Speed },
			{ "Test L Duty    ", MotorL_Test_Duty },
			{ "Test R Duty    ", MotorR_Test_Duty },
			{ "Mark Live Test ", Mark_Live_Test },
			{ "Drive Data     ", Print_Drive_Data },
			{ "Drive Data Buf ", Print_Drive_Data_Buffer },
//			{ "Test Velocity  ", Motor_Test_Velocity },
//			{ "Switch Test    ", Switch_Test },
	};

	uint8_t sw = 0;
	uint8_t count = 0;
	uint8_t menuCnt = sizeof(menus) / sizeof(t_menuData);

	Custom_OLED_Clear();
	while(1) {
		Custom_OLED_Clear();
		while(CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {
			Custom_OLED_Printf("%s", menus[count].menuName);

			if (sw == CUSTOM_SW_1) {
				if (count == 0)
					count = menuCnt - 1;
				else
					count--;
			}
			else if (sw == CUSTOM_SW_2) {
				if (count == menuCnt - 1)
					count = 0;
				else
					count++;
			}
		}
		menus[count].func();
	}
	Custom_OLED_Clear();
}
