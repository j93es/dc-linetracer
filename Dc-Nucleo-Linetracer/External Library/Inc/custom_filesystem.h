/*
 * custom_filesystem.h
 *
 *  Created on: May 11, 2021
 *      Author: Joonho Gwon
 */

#ifndef INC_CUSTOM_FILESYSTEM_H_
#define INC_CUSTOM_FILESYSTEM_H_

#include "custom_flash.h"
#include <stdbool.h>

#define FILENAME_LENGTH 20					// 파일의 이름 최대 길이

typedef struct
{
	char filename[FILENAME_LENGTH];
	uint32_t size;
	uint8_t checksum;
} FileInfo_t;

#define FILEINFO_NULL		((FileInfo_t*)0)	// 잘못된 파일을 나타내는 구조체
#define FILEINFO_SIZE 		sizeof(FileInfo_t)	// FileInfo_t 구조체의 크기
#define FILESYSTEM_SIZE 	0x1000				// 전체 파일시스템의 크기 (4096 byte)

void Custom_FileSystem_Load();
bool Custom_FileSystem_Flush();
FileInfo_t* Custom_FileSystem_Find(char *filename);
bool Custom_FileSystem_Delete(FileInfo_t *file);
bool Custom_FileSystem_Read(FileInfo_t *file, uint8_t *dest, uint32_t length);
bool Custom_FileSystem_Write(char *filename, uint8_t *data, uint32_t length);
void Custom_FileSystem_Reset();

#endif /* INC_CUSTOM_FILESYSTEM_H_ */
