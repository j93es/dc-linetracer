/*
 * custom_flash.c
 *
 *  Created on: May 11, 2021
 *      Author: Joonho Gwon
 */

#include "custom_flash.h"
#include "custom_exception.h"

/*
 * Flash의 Sector 3에 데이터를 쓰거나 읽는 라이브러리.
 * 읽기, 쓰기 단위는 1byte이다.
 * 만약 정상적으로 읽기, 쓰기가 잘 이루어졌다면 CUSTOM_FLASH_SUCCESS를 반환하고, 그렇지 않다면 에러를 나타내는 문자열을 반환한다.
 */

bool Custom_Flash_Check_Error() {
	uint32_t fsr = FLASH->SR;
	ASSERT_MSG(!(fsr & FLASH_SR_PGAERR), "Programming sequence error.");
	ASSERT_MSG(!(fsr & FLASH_SR_PGPERR), "Programming parallelism error.");
	ASSERT_MSG(!(fsr & FLASH_SR_PGAERR), "Programming alignment error.");
	ASSERT_MSG(!(fsr & FLASH_SR_WRPERR), "Write protection error.");

	return true;
}

bool Custom_Flash_Read(uint8_t *data, uint32_t length)
{
	ASSERT_MSG(length < CUSTOM_FLASH_SIZE, "Data is too large.");
	for (int i = 0; i < length; i++) {
		uint32_t address = CUSTOM_FLASH_BASE + i;
		data[i] = *(__IO uint8_t*) (address);
	}
	return Custom_Flash_Check_Error();
}

bool Custom_Flash_Unlock() {
	/*
	 * FLASH_CR 레지스터는 플래시를 제어하는 레지스터인데, 기본적으로 쓰기가 불가능하다.
	 * 이는 우연한 전기적 노이즈 등으로 인해 플래시가 지워지는 등의 오류를 방지하기 위해서다.
	 * FLASH_CR레지스터에 값을 쓸 수 있게 하려면 FLASH_KEYR 레지스터에 하드웨어적으로 지정된
	 * 키값을 연속으로 쓰면 된다.
	 */

	ASSERT_MSG(FLASH->CR & FLASH_CR_LOCK, "Flash is already unlocked.");

	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	ASSERT_MSG(!(FLASH->CR & FLASH_CR_LOCK), "Flash is not unlocked.");

	return Custom_Flash_Check_Error();
}

void Custom_Flash_Lock() {
	FLASH->CR |= FLASH_CR_LOCK;
}

void Custom_Flash_Wait() {
	// 이전 작업이 진행 중인지 BSY(Busy)비트를 검사해서 기다린다.
	while (FLASH->SR & FLASH_SR_BSY);
}

bool Custom_Flash_Erase() {
	Custom_Flash_Wait();

	ASSERT(Custom_Flash_Unlock());

	// 한 번에 몇 비트씩 쓰거나 지울지 설정.
	// 한 바이트 단위는 0x00을 쓰면 되므로, PSIZE 비트들을 클리어해주기만 하면 된다.
	FLASH->CR &= ~FLASH_CR_PSIZE;

	// 플래시를 지우기 위한 SER(Section ERase)비트 설정
	FLASH->CR |= FLASH_CR_SER;

	// 어떤 플래시를 지울 것인지 선택하기 위해 SNB(Sector NumBer)비트 설정
	FLASH->CR &= ~FLASH_CR_SNB;
	FLASH->CR |= CUSTOM_FLASH_SECTOR << FLASH_CR_SNB_Pos;

	// 지우는 작업을 시작하기 위해 STRT(STaRT)비트 설정
	FLASH->CR |= FLASH_CR_STRT;

	Custom_Flash_Wait();
	Custom_Flash_Lock();

	return Custom_Flash_Check_Error();
}

bool Custom_Flash_Write(uint8_t *src, uint32_t length)
{
	Custom_Flash_Wait();
	if (!Custom_Flash_Unlock()) return false;

	// 한 번에 몇 비트씩 쓰거나 지울지 설정
	FLASH->CR &= ~FLASH_CR_PSIZE;

	for (int i = 0; i < length; i++) {
		// 플래시를 쓰기 위한 PG(ProGram) 비트 설정
		FLASH->CR |= FLASH_CR_PG;

		// 실제 플래시에 데이터 기록
		*(__IO uint8_t*) (CUSTOM_FLASH_BASE + i) = src[i];

		// 데이터 기록이 끝날 때까지 대기
		Custom_Flash_Wait();
	}

	// 메뉴얼엔 나와 있지 않지만, 혹시 모르니 PG 비트 클리어.
	FLASH->CR &= ~FLASH_CR_PG;

	Custom_Flash_Wait();
	Custom_Flash_Lock();

	return Custom_Flash_Check_Error();
}
