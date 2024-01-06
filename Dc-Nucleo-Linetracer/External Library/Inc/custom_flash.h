/*
 * custom_flash.h
 *
 *  Created on: May 11, 2021
 *      Author: Joonho Gwon
 */

#ifndef INC_CUSTOM_FLASH_H_
#define INC_CUSTOM_FLASH_H_

#include "main.h"
#include <stdbool.h>

#define CUSTOM_FLASH_SECTOR		3U			// Sector 3
#define CUSTOM_FLASH_BASE		0x0800C000	// Base address of sector 3
#define CUSTOM_FLASH_SIZE 		0x4000		// Size of sector 3 (in bytes)

bool Custom_Flash_Read(uint8_t *data, uint32_t length);

bool Custom_Flash_Erase();

bool Custom_Flash_Write(uint8_t *src, uint32_t length);

#endif /* INC_CUSTOM_FLASH_H_ */
