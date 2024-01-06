/*
 * custom_exception.h
 *
 *  Created on: May 18, 2021
 *      Author: Joonho Gwon
 */

#ifndef INC_CUSTOM_EXCEPTION_H_

#define CUSTOM_EXCEPTION_MSG_NONE ((char*)0)

// 에러 메시지를 화면에 영원히 출력한다. 반드시 보드를 재시작해야한다. 아주 치명적인 에러가 발생했을 경우 호출하면 된다.
void Custom_Exception_Halt();
// 에러 메시지를 화면에 표시하기는 하는데, 아무 버튼이나 누르면 나가진다.
void Custom_Exception_Pause();

// 이 함수들은 직접 사용할 수 있지만, 굳이 그럴 필요가 없다.
void __Custom_Exception_Set_Message(char* file, int line, char *msg);
char* __Custom_Exception_Get_Message();

// 위 함수들을 직접 사용하는 대신 이 매크로 함수를 사용하면 편리하다.
// 이 매크로 함수들을 사용하면 자동으로 에러가 어떤 라인에서, 또 어떤 파일에서 발생했는지 알려준다.

// 에러 메시지를 설정한다.
#define SET_ERR(msg) __Custom_Exception_Set_Message(__FILE__,__LINE__,msg)
// 에러 메시지를 가져온다.
#define GET_ERR(msg) __Custom_Exception_Get_Message()

// 아래 매크로 함수들은 단순히 코드 길이를 줄이고 가독성을 높이기 위해서 만든 것들이다.

// 만약 조건을 검사해서 조건이 false이면 false를 반환한다.
#define ASSERT(condition) {if(!(condition)) return false;}
// 만약 조건을 검사해서 조건이 false이면 에러 메시지를 설정하고 false를 반환한다.
#define ASSERT_MSG(condition, msg) {if(!(condition)){SET_ERR(msg);return false;}}

#define INC_CUSTOM_EXCEPTION_H_

#endif /* INC_CUSTOM_EXCEPTION_H_ */
