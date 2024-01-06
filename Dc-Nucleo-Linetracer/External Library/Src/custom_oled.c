/*
 * custom_oled.c
 * SSD1331 SPI Driver
 *
 * [ OLED Pins ]
 * GND - GND
 * VCC - 5V
 * SCL - PB10 (SPI2)
 * SDA - PB15 (SPI2)
 * RES - NRST
 *       OLED 초기화하는데 사용한다. Low 신호가 들어올 때 초기화된다.
 * DC  - PA4  (GPIO)
 *       High 신호일 때 Data 모드로 동작하고, Low 신호일 때 Command 모드로 동작한다.
 * CS  - PB2  (GPIO)
 *       CS는 Chip Select의 약자로 SPI에서 슬레이브를 선택할 때 사용하는 핀이다.
 *       Low 신호일 때 SPI 신호가 수용되며, High 신호일 때는 무시된다.
 *
 * [ OLED GDDRAM ]
 * Graphic Display Data RAM(GDDRAM)은 OLED 화면에 표시될 데이터를 보관하는 저장소이다.
 * GDDRAM의 크기는 96 x 64 x 16bits이다. (가로 96픽셀, 세로 64픽셀, 픽셀 당 16비트)
 *
 * SSD1331의 GDDRAM은 16비트만 사용하고, 색상 A, B, C를 정의하자. (색상 A, B, C가 무엇을 의미하는 지는 나중에 서술)
 * 16비트 중에서 5비트는 색상 A, 6비트는 색상 B, 남은 5비트는 색상 C로 사용한다.
 * 16비트로 A, B, C 색상을 표시하는 것을 다음과 같이 정의하자.
 *   Bit: 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *   Map: C4 C3 C2 C1 C0 B5 B4 B3 B2 B1 B0 A4 A3 A2 A1 A0
 *
 * 우리는 일상적으로 24비트 컬러를 사용한다. 각각 다음과 같이 나타낼 수 있다.
 *           7  6  5  4  3  2  1  0
 *   Red:   R7 R6 R5 R4 R3 R2 R1 R0
 *   Green: G7 G6 G5 G4 G3 G2 G1 G0
 *   Blue:  B7 B6 B5 B4 B3 B2 B1 B0
 *
 * SSD1331은 5~6비트만 사용하므로, 색 표현에서 가장 중요한 상위 비트 몇 개만 사용한다.
 * 이 중 빨강은 R3~R7, 총 5비트만, 초록은 G2~G7, 총 6비트, 파랑은 B3~B7, 총 5비트를 사용한다.
 *   Bit: 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 *   Map: R7 R6 R5 R4 R3 G7 G6 G5 G4 G3 G2 B7 B6 B5 B4 B3
 *
 * DC 핀이 High일 때, SPI 통신으로 데이터를 전송하면 GDDRAM에 기록된다.
 * 기록되는 위치는 내부적으로 상태가 보존되며, 데이터 하나가 기록되면 그 다음 데이터를 알아서 가리키도록 설계돼있다.
 * 이때 다음 비트를 선택하는 기준은 세로(다음 열)가 될 수 있고, 가로(다음 행)가 될 수 있다.
 * 우리의 폰트는 가로로 읽는 것이 편하기 때문에 가로(다음 행)으로 다음 픽셀을 선택하도록 설정했다.(OLED_CMD_SETREMAP).
 *
 *  Created on: Jun 27, 2023
 *      Author: Seongho Lee
 *
 *    Modified: July 7, 2023
 *      Author: Pierre de Starlit
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "custom_gpio.h"

#include "starfont.h"
#define FONT		STARFONT_ASCII
#define FONT_WIDTH	STARFONT_ASCII_WIDTH
#define FONT_HEIGHT	STARFONT_ASCII_HEIGHT

#include "custom_oled.h"
#define OLED_BUFFER_SIZE 	512
#define OLED_SPI			SPI2
#define OLED_WIDTH			96
#define OLED_HEIGHT			64

enum OLED_Commands {
	OLED_CMD_SETCOLUMN = 0x15, // Set column address
	OLED_CMD_DRAWLINE = 0x21, // Draw line
	OLED_CMD_DRAWRECT = 0x22, // Draw rectangle
	OLED_CMD_CLEAR = 0x25, // Clear window
	OLED_CMD_FILL = 0x26, // Fill enable/disable
	OLED_CMD_SETROW = 0x75, // Set row adress
	OLED_CMD_CONTRASTA = 0x81, // Set contrast for color A
	OLED_CMD_CONTRASTB = 0x82, // Set contrast for color B
	OLED_CMD_CONTRASTC = 0x83, // Set contrast for color C
	OLED_CMD_MASTERCURRENT = 0x87, // Master current control
	OLED_CMD_PRECHARGEA = 0x8A, // Set second pre-charge speed for color A
	OLED_CMD_PRECHARGEB = 0x8B, // Set second pre-charge speed for color B
	OLED_CMD_PRECHARGEC = 0x8C, // Set second pre-charge speed for color C
	OLED_CMD_SETREMAP = 0xA0, // Set re-map & data format
	OLED_CMD_STARTLINE = 0xA1, // Set display start line
	OLED_CMD_DISPLAYOFFSET = 0xA2, // Set display offset(Set vertical offset by Com)
	OLED_CMD_NORMALDISPLAY = 0xA4, // Set display to normal mode
	OLED_CMD_DISPLAYALLON = 0xA5, // Set entire display ON
	OLED_CMD_DISPLAYALLOFF = 0xA6, // Set entire display OFF
	OLED_CMD_INVERTDISPLAY = 0xA7, // Invert display
	OLED_CMD_SETMULTIPLEX = 0xA8, // Set multiplex ratio
	OLED_CMD_SETMASTER = 0xAD, // Set master configuration
	OLED_CMD_DISPLAYOFF = 0xAE, // Display OFF (sleep mode)
	OLED_CMD_DISPLAYON = 0xAF, // Normal Brightness Display ON
	OLED_CMD_POWERMODE = 0xB0, // Power save mode
	OLED_CMD_PRECHARGE = 0xB1, // Phase 1 and 2 period adjustment
	OLED_CMD_CLOCKDIV = 0xB3, // Set display clock divide ratio/oscillator frequency
	OLED_CMD_PRECHARGELEVEL = 0xBB, // Set pre-charge voltage
	OLED_CMD_VCOMH = 0xBE, // Set Vcomh voltge
};

/**
 * @brief SPI 통신으로 OLED에 데이터를 전송하는 함수.
 *        매개변수로 전달 받은 데이터의 전송이 완료될 때까지 기다린다. 즉, 동기적(synchronous)으로 동작한다.
 *
 * @param cmd 보낼 명령어의 배열
 * @param length 보낼 명령어들의 총 길이
 */
static inline void Custom_OLED_SPI_Tramsnit_Blocking(const uint8_t data) {
	while (!LL_SPI_IsActiveFlag_TXE(OLED_SPI));
	LL_SPI_TransmitData8(OLED_SPI, data);
	while (LL_SPI_IsActiveFlag_BSY(OLED_SPI));
}

/**
 * @brief OLED에 명령을 보내는 함수.
 * 
 * @param cmd 보낼 명령어의 배열
 * @param length 보낼 명령어들의 총 길이
 */
static inline void Custom_OLED_Write_Command(const uint8_t *cmd, size_t length) {
	Custom_GPIO_Set(OLED_DC_GPIO_Port, OLED_DC_Pin, 0); // 0 is command mode
	Custom_GPIO_Set(OLED_CS_GPIO_Port, OLED_CS_Pin, 0);

	for (int i = 0; i < length; i++) {
		Custom_OLED_SPI_Tramsnit_Blocking(cmd[i]);
	}

	Custom_GPIO_Set(OLED_CS_GPIO_Port, OLED_CS_Pin, 1);
}

/**
 * @brief SPI 통신으로 OLED의 GDDRAM에 값(색상 데이터)을 작성하는 함수.
 *        매개변수로 전달 받은 데이터를 모두 보낼 때까지 동기적(synchronous)으로 동작한다.
 *
 * @param data 16비트 형태의 색상값들의 배열
 * @param length 보낼 값들의 갯수 (바이트 수가 아님을 주의)
 */
static inline void Custom_OLED_Write_Data(const uint16_t *data, size_t length){
	Custom_GPIO_Set(OLED_DC_GPIO_Port, OLED_DC_Pin, 1); // 1 is data mode
	Custom_GPIO_Set(OLED_CS_GPIO_Port, OLED_CS_Pin, 0);

	/**
	 * SSD1331 GDDRAM에 데이터를 어떻게 보내야 하는지 알아보자.
	 * 우리는 SSD1331을 초기화할 때 Remap & Color Depth setting에서 color format을 65k format 1 모드로 설정했다.
	 * 65k format 1모드는 다음과 같이 데이터를 전송하라고 정의한다.
	 *   Bit: 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	 *   1st:  x  x  x  x  x  x  x  x C4 C3 C2 C1 C0 B5 B4 B3
	 *   2nd:  x  x  x  x  x  x  x  x B2 B1 B0 A4 A3 A2 A1 A0
	 *
	 * 우리가 사용하는 MCU는 Cortex-M4 시리즈이며, little endian으로 데이터를 저장한다.
	 * 하지만, SSD1331은 big endian 방식으로 데이터를 처리하므로, little-big 사이의 변환이 필요하다.
	 * 이를테면, 모든 A 비트가 1인 상황인 0x001F([15:0] 0000 0000 0001 1111)는
	 * MCU 메모리 어딘가 첫 번째 번지에 0x1F 값이 들어가고, 그 다음 번지에 0x00이 들어간다.
	 * 그러면, SPI 통신으로 데이터를 8비트씩 보낼 때 0번지 부터 값을 보낼 것이므로 0x1F을 보내고 0x00을 그 다음으로 보낼 것이다.
	 * SSD1331은 C1 C0 B5 B4 B3 데이터가 1인 것으로 인식할 것이며 이러면 안되므로,
	 * 상위 8비트 0x00을 보내고 나서 하위 8비트 0x1F 보내는 식으로 진행해야 한다.
	 * 다른 색상 정보도 마찬가지로 데이터를 전송할 때 상위 8비트를 먼저 보내야 한다.
	 */
	for (int i = 0; i < length; i++) {
		Custom_OLED_SPI_Tramsnit_Blocking(data[i] >> 8);   // 상위 8비트 전송
		Custom_OLED_SPI_Tramsnit_Blocking(data[i] & 0xFF); // 하위 8비트 전송
	}

	Custom_GPIO_Set(OLED_CS_GPIO_Port, OLED_CS_Pin, 1);
}

/**
 * @brief SSD1331 OLED의 GDDRAM에 값을 쓸 때 값이 쓰여지는 범위를 지정하는 함수.
 *        SSD1331 OLED에는 GDDRAM이라는 그래픽 메모리가 존재하며, Data 모드일 경우 SPI 통신으로 값을 쓸 때 GDDRAM에 값이 쓰여진다.
 *        이 함수는 값이 쓰여지는 범위를 지정할 수 있는데, 이 범위는 그래픽 메모리에 값이 쓰여지는 범위를 설정한다.
 *        범위를 설정하게 되면, 그래픽 메모리에 수직적으로(위에서 아래로) 값을 기록하다가 설정된 범위를 넘어갔을 때, 다음 열로(왼쪽에서 오른쪽으로) 이동하여 계속해서 값이 기록되는 부가 효과를 만들 수 있다.
 * 
 * @param x1 GDDRAM 시작 열
 * @param y1 GDDRAM 시작 행
 * @param x2 GDDRAM 종료 열
 * @param y2 GDDRAM 종료 행
 */
static inline void Custom_OLED_Set_Window(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
	const uint8_t cmd[] = {
		OLED_CMD_SETCOLUMN,
		x1, x2,
		OLED_CMD_SETROW,
		y1, y2,
	};

	Custom_OLED_Write_Command(cmd, sizeof(cmd));
}

void Custom_OLED_Clear() {
	uint16_t data = 0x0000;

	Custom_OLED_Set_Window(0, 0, OLED_WIDTH - 1, OLED_HEIGHT - 1);

    for (int i = 0; i < OLED_WIDTH * OLED_HEIGHT; i++) {
        Custom_OLED_Write_Data(&data, 1);
    }
}

void Custom_OLED_Init() {
	// SPI 활성화
	LL_SPI_SetTransferDirection(OLED_SPI, LL_SPI_HALF_DUPLEX_TX);
	LL_SPI_Enable(OLED_SPI);

	/*
	 * 우리가 사용하는 SSD1331 OLED 모듈의 초기화 명령어들이다.
	 * 아래의 사이트에서 해당 명령어를 참조했으며, SSD1331 데이터시트 8장과 9절을 참조하면 더욱 자세한 내용을 알 수 있다.
	 * https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	const uint8_t init_cmd[] = {
		OLED_CMD_DISPLAYOFF,
		/*
		 * 0x72 = 0b01110010 -> Horizontal address increment
		 * 0x73 = 0b01110011 -> Vertical address increment (이 모드를 사용, 제공된 폰트가 위에서 아래로 읽는 형태이기 때문)
		 */
		OLED_CMD_SETREMAP, 0x73,
		OLED_CMD_STARTLINE, 0x00,
		OLED_CMD_DISPLAYOFFSET, 0x00,
		OLED_CMD_NORMALDISPLAY,
		OLED_CMD_SETMULTIPLEX, 0x3F,
		OLED_CMD_SETMASTER, 0x8E,
		OLED_CMD_POWERMODE, 0x0B,
		OLED_CMD_PRECHARGE, 0x31,
		OLED_CMD_CLOCKDIV, 0xF0,
		OLED_CMD_PRECHARGEA, 0x64,
		OLED_CMD_PRECHARGEB, 0x78,
		OLED_CMD_PRECHARGEC, 0x64,
		OLED_CMD_PRECHARGELEVEL, 0x3A,
		OLED_CMD_VCOMH, 0x3E,
		OLED_CMD_MASTERCURRENT, 0x06,
		OLED_CMD_CONTRASTA, 0x91,
		OLED_CMD_CONTRASTB, 0x50,
		OLED_CMD_CONTRASTC, 0x7D,
		OLED_CMD_DISPLAYON
	};

	Custom_OLED_Write_Command(init_cmd, sizeof(init_cmd));
	Custom_OLED_Clear();
}

int Custom_OLED_Putchar(char c, uint16_t color, uint8_t x, uint8_t y, uint8_t size) {
	uint8_t xe = x + FONT_WIDTH * size - 1;
	uint8_t ye = y + FONT_HEIGHT * size - 1;

	if (xe >= OLED_WIDTH || ye >= OLED_HEIGHT) {
		// OLED 화면을 넘어서면 유효하지 않은 명령이다.
		return -1;
	}
	// OLED GDDRAM에 데이터를 넣을 시작 위치(x, y)와 종료 위치(xe, ye)를 설정한다.
	Custom_OLED_Set_Window(x, y, xe, ye);

	const char *font = FONT[(int)c];
	static uint16_t data[FONT_WIDTH * FONT_HEIGHT * 4];

	for (int i = 0; i < FONT_WIDTH; i++) {
		for (int j = 0; j < FONT_HEIGHT; j++) {
			for(int x = 0; x < size; x++){
				for(int y = 0; y < size; y++){
			    	data[FONT_HEIGHT * size * (i*size+ x) + j*size + y] = font[i] & (1 << j) ? color : 0x0000;
			    }
			}
		}
	}

	Custom_OLED_Write_Data(data, sizeof(data) / sizeof(uint16_t) / 4 * size * size);
	return 0;
}

void Custom_OLED_Printf(const char *format, ...) {
	char buffer[OLED_BUFFER_SIZE];
	uint8_t posX = 0;
	uint8_t posY = 0;
	
	/*
	 * printf 함수처럼 가변 인자를 받고 vsprintf 함수를 통해 buffer에 서식 문자가 적용된 문자열을 저장한다.
	 */
	va_list args;
	va_start(args, format);
	vsprintf(buffer, format, args);
	va_end(args);

	/*
	 * Pierre de Starlit(P. J. Kim)의 OLED 서식 문자 표준을 따라 처리한다.
	 */
	int cursor = 0;
	uint16_t color = OLED_COLOR_WHITE;
	uint8_t size = 1;

	while (buffer[cursor]) {

		// 서식 문자의 시작을 나타내는 문자
		if (buffer[cursor] == '/') {
			char nextChar = buffer[cursor + 1];

			if (nextChar == '/') {
				cursor += 1;

			} else {
				switch (nextChar) {
				// 문자를 표시할 행을 지정하는 서식 문자
				case '0': case '1': case '2': case '3': case '4': case '5': case '6':
					posX = 0;
					posY = (FONT_HEIGHT + 1) * (nextChar - '0'); // FONT_HEIGHT + 1를 해주는 이유는, 폰트 아래 한 칸 여백을 남겨두기 위함이다.
					break;
				// 색상을 지정할 서식 문자
				case 'w': color = OLED_COLOR_WHITE;   break;
				case 'r': color = OLED_COLOR_RED;     break;
				case 'g': color = OLED_COLOR_GREEN;   break;
				case 'b': color = OLED_COLOR_BLUE;    break;
				case 'y': color = OLED_COLOR_YELLOW;  break;
				case 'c': color = OLED_COLOR_CYAN;    break;
				case 'm': color = OLED_COLOR_MAGENTA; break;
				case 'o': color = OLED_COLOR_ORANGE;  break;
				case 'l': color = OLED_COLOR_LIME;    break;
				case 't': color = OLED_COLOR_MINT;    break;
				case 's': color = OLED_COLOR_SEA;     break;
				case 'v': color = OLED_COLOR_VIOLET;  break;
				case 'p': color = OLED_COLOR_ROSE;    break;
				case 'K': color = OLED_COLOR_GRAY;    break;
				case 'k': color = OLED_COLOR_BLACK;   break;
				// TODO: 크기 지정 문자 추가
				case 'A': size = 2;                   break;
				case 'a': size = 1;                   break;
				case '#':
                    {
                        uint32_t colordata = 0;
                        for(uint32_t i = 0; i < 6; i++){
                            if(buffer[cursor + i + 2] >= 'A' && buffer[cursor + i + 2] <= 'F'){
                            	colordata *= 16;
                            	colordata += buffer[cursor + i + 2] - 'A' + 10;
                            }
                            else if(buffer[cursor + i + 2] >= 'a' && buffer[cursor + i + 2] <= 'f'){
                            	colordata *= 16;
                            	colordata += buffer[cursor + i + 2] - 'a' + 10;
                            }
                            else if(buffer[cursor + i + 2] >= '0' && buffer[cursor + i + 2] <= '9'){
                            	colordata *= 16;
                            	colordata += buffer[cursor + i + 2] - '0';
                            }
                        }
                        color = ((colordata >> 8) & 0xF800) | ((colordata >> 5) & 0x07F0) | ((colordata >> 3) & 0x001F);

                    }
                cursor += 6;
                break;
				}

				cursor += 2;
				continue;
			}
		}

		Custom_OLED_Putchar(buffer[cursor], color, posX, posY, size); // 문자를 화면에 표시한다.
		posX += (FONT_WIDTH + 1) * size; // FONT_WIDTH + 1를 해주는 이유는, 폰트 오른쪽 한 칸 여백을 남겨두기 위함이다.
		cursor++;
	}
}
