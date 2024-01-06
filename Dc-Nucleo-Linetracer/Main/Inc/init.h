/*
 * init.h
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_



#include <stdint.h>
#include <stdbool.h>


// 공용 매크로
#define CUSTOM_FALSE				0
#define CUSTOM_TRUE					1
#define ABS(x) 						( ((x) < 0) ? (-1 * (x)) : (x) )
#define GET_MIN(x, y)				( ((x) < (y)) ? (x) : (y) )


#define MAX_OLED_LEN	15




void	Init(void);





typedef struct	s_menuData
{
	char menuName[MAX_OLED_LEN];
	void (*func)(void);
}				t_menuData;







#endif /* INC_INIT_H_ */
