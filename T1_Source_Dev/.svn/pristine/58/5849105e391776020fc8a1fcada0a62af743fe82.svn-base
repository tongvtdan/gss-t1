

#ifndef __T1_ROBOT
#define __T1_ROBOT

#include "stm32f10x.h"

#define DISABLE 				0
#define ENABLE 				1

#define SYSTEM_STOP			0
#define SYSTEM_START			1

#define BED_DETECT			0
#define BED_NOTDETECT		1

#define G1_NOT_ON_T1			0
#define G1_ON_T1				1
#define G1_WORKING			2
#define G1_NOTWORKING		3

#define EDGE_DETECT			1
#define EDGE_NOTDETECT		0

#define DIR_FORWARD			1
#define DIR_BACKWARD		0

#define RULO_FORWARD		1
#define RULO_BACKWARD		0

#define SUBRULO_RADIUS		2    // 2 cm
#define MAX_HOSE_LENGTH		50	   //cm

typedef struct{
	uint8_t sys_status;	   	//trang thai hien tai cua he thong
	uint8_t bed_status;
	uint8_t edge_status;
	uint8_t rulo_status;
	uint32_t hose_length;   //chieu dai day da di ra o thoi diem hien tai
	uint8_t G1_on_T1_status;
	uint8_t G1_working_status;
}T1_ROBOT;


void T1_Run(uint8_t Dir);
void T1_Stop(void);
void Lazer_Stop(void);
void Lazer_Run(uint8_t Dir);
void Lazer_Home(T1_ROBOT* robot);
void Rulo_Run(T1_ROBOT* robot, uint8_t Dir);
void Rulo_Stop(void);

void G1_Release_Cmd(T1_ROBOT* robot);		//
void G1_Detect_Cmd(void);					//

void System_Init(T1_ROBOT* robot);

#endif /* __T1_ROBOT_ */

