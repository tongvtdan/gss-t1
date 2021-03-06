/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "T1Robot_lib.h"
#include <stdio.h>
#include <math.h>


/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
void RCC_Configuration(void);
void TimingDelay_Decrement(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);
void USART_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);


T1_ROBOT T1Robot;

uint8_t lazer_dir=0;
uint8_t input_changed;
static __IO uint32_t TimingDelay, index=0;
uint32_t  encoder_value =0 ;
uint16_t encoder_count = 0;
int16_t encodertemp=0;
uint8_t lazer_count=0;
uint16_t icount = 0;

PUTCHAR_PROTOTYPE
{
	//Place your implementation of fputc here 
	//e.g. write a character to the USART 
	USART_SendData(USART1, (uint8_t) ch);

	//Loop until the end of transmission 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
		
	return ch;
}

int main(void)
{	
 	RCC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
    	TIM_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();

	System_Init(&T1Robot);	
	
	while (1)
	{	
		/*
		index++;
		encoder_count = abs(TIM_GetCounter(TIM3)-30000);
		*/
		
		if(T1Robot.sys_status == SYSTEM_START)	 // he thong dc phep hoat dong
		{
			if(T1Robot.edge_status == EDGE_DETECT)
			{								
				lazer_dir=!lazer_dir;	
				Lazer_Run(lazer_dir);
				T1Robot.edge_status = EDGE_NOTDETECT;
			}
		/*
		//printf("G1_status %d\n\r",T1Robot.G1_working_status);	
			
			if(T1Robot.G1_on_T1_status == G1_ON_T1)
			{
				if(T1Robot.G1_working_status == G1_NOTWORKING)
				{
					//Clear_Message_Display();
					if(T1Robot.bed_status == BED_DETECT)
					{
						T1_Stop();
						G1_Release_Cmd(&T1Robot);
						Delay(10);
						G1_Detect_Cmd();	
						T1Robot.edge_status = EDGE_DETECT;
						while(T1Robot.G1_on_T1_status == G1_ON_T1);
						
						encoder_value = 0;
						T1Robot.hose_length = 0;
						encodertemp = 0;
						icount = 0;
						
						Rulo_Run(&T1Robot, RULO_FORWARD);
					}
					else 	if(T1Robot.bed_status == BED_NOTDETECT)
					{
						Lazer_Stop();
						Rulo_Stop();
						Delay(500);
						T1_Run(DIR_FORWARD);
					}
				}
				else if(T1Robot.G1_working_status == G1_WORKING)
				{
					T1Robot.G1_working_status = G1_NOTWORKING;
					T1Robot.bed_status = BED_NOTDETECT;
				}
			}
			else	if(T1Robot.G1_on_T1_status == G1_NOT_ON_T1)
			{
				if(T1Robot.G1_working_status == G1_WORKING)
				{
					if(T1Robot.edge_status == EDGE_DETECT)
					{								
						lazer_dir=!lazer_dir;	
						Lazer_Run(lazer_dir);
						T1Robot.edge_status = EDGE_NOTDETECT;
					}
				}
			}
			
						
			if((index%100) == 0)
			{
				if(T1Robot.G1_working_status == G1_WORKING)
				{
					encoder_value = (uint32_t)(abs(encodertemp)*30000+encoder_count);
					T1Robot.hose_length = (encoder_value/200)*2*3.14159*SUBRULO_RADIUS;	// cm unit
					printf("encoder value %d\trulo_dir %d\n\r",T1Robot.hose_length, T1Robot.rulo_status);
					if(T1Robot.hose_length >= MAX_HOSE_LENGTH)
					{		
							TIM_SetCounter(TIM3,30000);				
							encoder_value = 0;
							T1Robot.hose_length = 0;
							encodertemp = 0;
							icount = 0;
							
						if(T1Robot.rulo_status == RULO_FORWARD)
						{
							Rulo_Stop();
							Delay(300);	

							Rulo_Run(&T1Robot, RULO_BACKWARD);
						}
						
						else if(T1Robot.rulo_status == RULO_BACKWARD)
						{
							Rulo_Stop();
							Lazer_Stop();
						}	
						
					}
				}
			}
			*/
			
		}
		
		else if(T1Robot.sys_status == SYSTEM_STOP)	 // he thong ngung hoat dong
		{
			Lazer_Stop();
			T1_Stop();
			Rulo_Stop();
			T1Robot.G1_on_T1_status = G1_NOT_ON_T1;
			T1Robot.G1_working_status = G1_NOTWORKING;
			T1Robot.bed_status = BED_NOTDETECT;
			T1Robot.edge_status = EDGE_DETECT;
		}



	}
}

void Delay(__IO uint32_t num)
{
	
	__IO uint32_t index = 0;

	// default system clock is 72MHz 
	for(index = (9000 * num); index != 0; index--)
	{
	}
		
	//TimingDelay = num;
	//while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
  /*
void TimingDelay_Decrement(void)
{	
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}
*/

void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_TIM1|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	/* Configure TIM3_CH1 and TIM3_CH2 pins for Encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PA9 for USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PA10 for USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB0 for input pull up */
	/* START BUTTON external interrupt pin PB0 */
	/* STOP BUTTON external interrupt pin PB1 */
	/* LIMIT SWITCH external interrupt pin PB5 */
	/* IR lighting sensor's external interrupt pin PB14 */		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PA12 for Led Indicator */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure TIM1_CH1 <-> PA8 pin PWM 38KHz for LAZER transmitter */
	/* Configure TIM1_CH4 <-> PA11 pin PWM 38KHz for IR led transmitter */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* lazer scan motor's direction control pins PB12, PB13*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* two control direction pin for rulo motor (mode PWM_CH1, PWM_CH2 Timer4) */	
	/* T1 Motor Moving control pins PB8, PB9 */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void USART_Configuration(void)
{
	USART_InitTypeDef 	USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

void TIM_Configuration(void)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;
	TIM_BDTRInitTypeDef  	 TIM_BDTRInitStructure;
	
/*----------------------------------------------------------------------------------------------*/
	/* Time1 configuration */
  	TIM_TimeBaseStructure.TIM_Period = 946;	  //TIM1 PWM frequency is 38KHz
  	TIM_TimeBaseStructure.TIM_Prescaler = 1;		 //clock rate for TIM1 is 36MHz
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: CH1,CH4 */
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 	TIM_OCInitStructure.TIM_Pulse = 850;						  
 	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* Automatic Output enable, Break, dead time and lock configuration*/	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 117;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
 
	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

/*----------------------------------------------------------------------------------------------*/
    	/* TIM3 configuration*/
	/* TIM3 config as Encoder Interface Mode to detect Rulo Position */
	TIM_TimeBaseStructure.TIM_Period = 60000;	        
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    	TIM_ICInitStructure.TIM_ICFilter = 0x5;//ICx_FILTER;
    	TIM_ICInit(TIM3, &TIM_ICInitStructure);
		

	/* enable TIM3 update event interrupt */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM3,30000); 	//reset counter value to 30000
	TIM_ARRPreloadConfig(TIM3, ENABLE);	
	/* TIM3 enable */ 
	TIM_Cmd(TIM3, ENABLE);
	
/*----------------------------------------------------------------------------------------------*/
	 //Time2configuration 
  	TIM_TimeBaseStructure.TIM_Period = 33332;	  //TIM2 PWM frequency is 15Hz
  	TIM_TimeBaseStructure.TIM_Prescaler = 143;	 //clock rate for TIM2 is 500KHz
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* enable TIM3 update event interrupt */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(TIM2, ENABLE);	
	// TIM2 enable counter 
  	TIM_Cmd(TIM2, ENABLE);	

/*----------------------------------------------------------------------------------------------*/
	/* Time4configuration */
  	TIM_TimeBaseStructure.TIM_Period = 4799;	  //TIM2 frequency is 7.5KHz
  	TIM_TimeBaseStructure.TIM_Prescaler = 1;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: CH1, CH2 */
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 	TIM_OCInitStructure.TIM_Pulse = 1000;						  
 	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	/* TIM3 enable counter */
  	TIM_Cmd(TIM4, ENABLE);	
}

void EXTI_Configuration(void)
{
	EXTI_InitTypeDef  EXTI_InitStructure;
	/* ------------------------------------------------------------------------------- */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);   //start
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);   //stop
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);   //limit switch
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14); //IR lighting sensor
	EXTI_ClearITPendingBit(EXTI_Line0|EXTI_Line1|EXTI_Line5|EXTI_Line14);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	/* NVIC config for EXTI_Line0 */	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* NVIC config for EXTI_Line1 */	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* NVIC config for EXTI_Line5 */	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* NVIC config for EXTI_Line14 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* NVIC config for TIM3 Update interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* NVIC config for TIM3 Update interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Lazer_Home(T1_ROBOT* robot)
{
	Lazer_Run(lazer_dir);
	Delay(300);
	if(robot->edge_status ==EDGE_DETECT) Lazer_Stop();
	else if(robot->edge_status !=EDGE_DETECT)
	{	
		lazer_dir=!lazer_dir;
		while(robot->edge_status !=EDGE_DETECT) Lazer_Run(lazer_dir);
	}
	Lazer_Stop();	
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
