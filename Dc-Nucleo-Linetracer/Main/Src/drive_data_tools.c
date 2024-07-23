/*
 * drive_data_tools.c
 */


#include "header_init.h"


void Print_Drive_Data() {
	uint32_t i = 1;
	uint8_t sw = 0;
	uint16_t markCnt_L = 0;
	uint16_t markCnt_R = 0;
	uint16_t crossCnt = 0;

	// 마크 개수 세기
	for (i = 1; driveData[i].markState != MARK_NONE; i += 1) {

		// 현재상태가 좌측 곡선인 경우
		if (driveData[i].markState == MARK_CURVE_L) {

			markCnt_L += 1;
		}

		// 현재상태가 우측 곡선인 경우
		else if (driveData[i].markState == MARK_CURVE_R) {

			markCnt_R += 1;
		}

		// 직선 (인덱스가 1부터 시작하기에 지정되지 않은 메모리에 접근하는 행동을 방지함)
		else if (driveData[i].markState == MARK_STRAIGHT) {

			// 이전 상태가 좌측 곡선이었을 경우
			if (driveData[i-1].markState == MARK_CURVE_L) {
				markCnt_L += 1;
			}
			// 이전 상태가 우측 곡선이었을 경우
			else if (driveData[i-1].markState == MARK_CURVE_R) {
				markCnt_R += 1;
			}
		}
	}

	for (i = 0; crossCntTable[i] != 0 && i < MAX_CROSS_CNT; i++) {
		crossCnt++;
	}


	// OLED에 변수명 변수값 출력
	Custom_OLED_Clear();
	Custom_OLED_Printf("/0mark L:   %d", markCnt_L);
	Custom_OLED_Printf("/1mark R:   %d", markCnt_R);
	Custom_OLED_Printf("/2cross:    %d", crossCnt);

	while (CUSTOM_SW_3 != Custom_Switch_Read());



	Custom_OLED_Clear();

	i = 0;

	while(CUSTOM_SW_3 != (sw = Custom_Switch_Read())) {

		if (driveData[i].markState == MARK_CURVE_L) {

			Custom_OLED_Printf("/0mark L");
		}
		else if (driveData[i].markState == MARK_CURVE_R) {

			Custom_OLED_Printf("/0mark R");
		}
		else if (driveData[i].markState == MARK_STRAIGHT) {

			Custom_OLED_Printf("/0straight");
		}

		Custom_OLED_Printf("/1L: %9u", driveData[i].tickCnt_L);
		Custom_OLED_Printf("/2R: %9u", driveData[i].tickCnt_R);
		Custom_OLED_Printf("/3C: %5u", driveData[i].crossCnt);

		if (sw == CUSTOM_SW_1) {

			i -= 1;
		}
		else if (sw == CUSTOM_SW_2) {

			i += 1;
		}

		if (driveData[i].markState == MARK_NONE) {
			break ;
		}
	}

	Custom_OLED_Clear();
}


/*
void Save_Drive_Data_Flash() {

	Custom_OLED_Clear();

	if (Custom_FileSystem_Write("drive data", (uint8_t *)driveData, sizeof(driveData)) == CUSTOM_TRUE) {
		Custom_OLED_Printf("SUCCESS");
	}
	else {
		Custom_OLED_Printf("SAVE FAIL");
	}

	while (CUSTOM_SW_BOTH != Custom_Switch_Read());

	Custom_OLED_Clear();
}



void Read_Drive_Data_Flash() {

	t_driveData driveDataBuffer[MAX_DRIVE_DATA_LEN];

	FileInfo_t *fileInfo = Custom_FileSystem_Find("drive data");

	Custom_OLED_Clear();

	if (fileInfo != FILEINFO_NULL && Custom_FileSystem_Read(fileInfo, (uint8_t *)driveDataBuffer, sizeof(driveDataBuffer)) == CUSTOM_TRUE) {

		for (uint32_t i = 0; i < MAX_DRIVE_DATA_LEN;  i++) {
			driveData[i] = driveDataBuffer[i];
		}
		Custom_OLED_Printf("SUCCESS");
	}
	else {
		Custom_OLED_Printf("READ FAIL");
	}

	while (CUSTOM_SW_BOTH != Custom_Switch_Read());

	Custom_OLED_Clear();
}

void Delete_Drive_Data_Flash() {

	FileInfo_t *fileInfo = Custom_FileSystem_Find("drive data");

	Custom_OLED_Clear();

	if (fileInfo != FILEINFO_NULL && Custom_FileSystem_Delete(fileInfo) == CUSTOM_TRUE) {
		Custom_OLED_Printf("SUCCESS");
	}
	else {
		Custom_OLED_Printf("DELETE FAIL");
	}

	while (CUSTOM_SW_BOTH != Custom_Switch_Read());

	Custom_OLED_Clear();
}
*/



