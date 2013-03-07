#include "T1Robot_lib.h"

extern uint8_t lazer_dir;
void T1_Run(uint8_t Dir)
{
	if(Dir == DIR_FORWARD)
	{		
		TIM_SetCompare3(TIM4, 0);		//PB8 <-> TIM4_CH3
		TIM_SetCompare4(TIM4, 3000);		//PB9 <-> TIM4_CH4
	}
	else if(Dir == DIR_BACKWARD)
	{	
		TIM_SetCompare3(TIM4, 3000);		//PB8 <-> TIM4_CH3
		TIM_SetCompare4(TIM4, 0);		//PB9 <-> TIM4_CH4
	}
}

void T1_Stop(void)
{
	TIM_SetCompare3(TIM4, 0);		//PB8 <-> TIM4_CH3
	TIM_SetCompare4(TIM4, 0);		//PB9 <-> TIM4_CH4
}

void Lazer_Run(uint8_t Dir)
{	
	if(Dir == DIR_FORWARD)
	{		
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	}
	else if(Dir == DIR_BACKWARD)
	{	
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
	}
	
	if(TIM_GetCapture1(TIM1) == 0)
	{
		TIM_SetCompare1(TIM1, 850);
	}
}

void Lazer_Stop(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);	

	if(TIM_GetCapture1(TIM1) == 850)
	{
		TIM_SetCompare1(TIM1, 0);
	}
}

void Rulo_Run(T1_ROBOT* robot, uint8_t Dir)
{
	if(Dir == RULO_FORWARD)
	{
		TIM_SetCompare1(TIM4, 0);		//PB6 <-> TIM4_CH1
		TIM_SetCompare2(TIM4, 1000);			//PB7 <-> TIM4_CH2

		robot->rulo_status = RULO_FORWARD;
	}
	else if(Dir == RULO_BACKWARD)
	{
		TIM_SetCompare1(TIM4, 2000);		//PB6 <-> TIM4_CH1
		TIM_SetCompare2(TIM4, 0);		//PB7 <-> TIM4_CH2
		
		robot->rulo_status = RULO_BACKWARD;
	}	
}
void Rulo_Stop(void)
{
	TIM_SetCompare1(TIM4, 0);		//PB6 <-> TIM4_CH1
	TIM_SetCompare2(TIM4, 0);		//PB7 <-> TIM4_CH2
}

void G1_Release_Cmd(T1_ROBOT* robot)
{
	TIM_SetCompare4(TIM1,0);		//PB7 <-> TIM4_CH2
	robot->G1_working_status = G1_WORKING;
}
void G1_Detect_Cmd(void)
{
	TIM_SetCompare4(TIM1,850);		//PB7 <-> TIM4_CH2
}

/* khoi tao cac trang thai ban dau cua he thong khi vua mo nguon */
void System_Init(T1_ROBOT* robot)
{
	robot->sys_status = SYSTEM_STOP;
	robot->bed_status = BED_NOTDETECT;
	robot->edge_status = EDGE_NOTDETECT;
	robot->hose_length = 0;				//chieu dai day da di ra ban dau la 0 cm	
	robot->G1_on_T1_status = G1_NOT_ON_T1;
	robot->G1_working_status = G1_NOTWORKING;

	T1_Stop();
	Rulo_Stop();
	Lazer_Home(robot);

	robot->G1_on_T1_status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);	
}

/* ---------------------------------------End file--------------------------------------------- */

