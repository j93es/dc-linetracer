/*
 * custom_exception.c
 *
 *  Created on: May 18, 2021
 *      Author: Joonho Gwon
 */

/*
 * 오류 처리를 위한 라이브러리이다.
 * 오류가 발생했을 때
 */

#include "custom_exception.h"
#include "custom_oled.h"
#include "custom_switch.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static char *_file;
static int _line;
static char *_message = CUSTOM_EXCEPTION_MSG_NONE;

void Custom_Exception_Reset() {
	_message = CUSTOM_EXCEPTION_MSG_NONE;
}

void __Custom_Exception_Set_Message(char *file, int line, char *message) {
	_file = file;
	_line = line;
	_message = message;
}

char* __Custom_Exception_Get_Message() {
	return _message;
}

void Custom_Exception_Halt() {
	if (_message == CUSTOM_EXCEPTION_MSG_NONE) return;

	// fullmsg는 이론상 static일 필요가 없으나, 실제로는 변수가 너무 길어 다른 메모리를 침범할 수 있어 안전하게 static으로 선언한다.
	static char fullmsg[100] = { 0 };
	sprintf(fullmsg, "%s at %s:%d ", _message, _file, _line);

	int msgLen = strlen(fullmsg);
	while (true) {
		for (int offset = 0; offset < msgLen; offset++) {
			char buffer[9] = { 0 };
			int i;
			for (i = 0; i < 8; i++) {
				buffer[i] = fullmsg[(offset + i) % msgLen];
				if (buffer[i] == 0) buffer[i] = ' ';
			}
			Custom_OLED_Clear();
			Custom_OLED_Printf("%s", buffer);
			Custom_Delay_ms(200);
		}
	}
}

void Custom_Exception_Pause() {
	// Custom_Exception_Halt와 똑같이 기능하되 아무 스위치나 누르면 빠져나가진다.

	if (_message == CUSTOM_EXCEPTION_MSG_NONE) return;

	static char fullmsg[100] = { 0 };
	sprintf(fullmsg, "%s at %s:%d ", _message, _file, _line);

	int msgLen = strlen(fullmsg);

	bool isDisplay = true;
	while (isDisplay) {
		for (int offset = 0; offset < msgLen; offset++) {
			char buffer[9] = { 0 };
			int i;
			for (i = 0; i < 8; i++) {
				buffer[i] = fullmsg[(offset + i) % msgLen];
				if (buffer[i] == 0) buffer[i] = ' ';
			}
			Custom_OLED_Clear();
			Custom_OLED_Printf("%s", buffer);
			Custom_Delay_ms(200);
			if (Custom_Switch_Read() != CUSTOM_SW_NONE) {
				isDisplay = false;
				break;
			}
		}
	}

	Custom_OLED_Clear();
}
