
#include "Encoder.h"
#include "main.h"
#include "Timer_1.h"
#include "Timer_2.h"
#include "Timer_8.h"
//#include "USART.h"
#include "string.h"
#include "stdio.h"
#include "fifo.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

#define SETBIT(x,y) (x |= (1 << y)) /* Set bit y in byte x*/
#define CLEARBIT(x,y) (x &= (~(1 << y))) /* Clear bit y in byte x*/


static int64_t TIM8_CNT = 0;
static volatile int count_overflow_TIM8 = 0; //overflow counter TIM8
static volatile int count_overflow = 0;			 //overflow counter TIM1
static int64_t Counter_PoluPeriod = 0;			 //half of duration one period
static int64_t Counter_Period = 0;					 //duration one period
static int Chan1_overflow_0 = 0;						 //
static int Chan1_overflow_00 = 0;
static int Chan2_overflow_0 = 0;
static int64_t counter_0 = 0;								 //TIM1 start value 
static int64_t counter_1 = 0;								 //value CCR1 TIM1
static int64_t counter_t = 0;								 //period clock duration

static float counter_t_float = 0;						 //duration of period, mcs
static float delta_t = 0;										 //period clock duration TIM1, mcs

float *ptr_buff_temp_float;									 //write buffer ptr
uint16_t Len_buff_temp_char = 0;						 //len wtite buffer
char buff_temp[64] = {0,};

static uint16_t aaa = 0;											//


FIFO( MK_TO_COMP_FIFO_SIZE ) MK_to_Comp_fifo = {0};
FIFO( COMP_TO_MK_FIFO_SIZE ) Comp_to_MK_fifo = {0};
FIFO( ERROR_FIFO_SIZE ) Error_fifo = {0};

uint32_t WIM_task; 

static uint16_t buff_1[64] = {0};
static uint16_t ss = 0;
static uint16_t rr = 0;

static uint16_t cnt_sector = 0;
#define NUM_REFRESH_SD_INFO 200
#define PERIOD_BLINK 20


void Encoder_Init (void){
	
	INTTIM_Config();		// init timer1, interrupt after overflow
	TIM1_Init();				// init timer1, channel1, input capture mode
	TIM2_Init();				// init timer1, channel3, input capture mode
	
//init status flags
	WIM_task &= ~WIM_MK_TO_COMP;					
	WIM_task &= ~WIM_INPUT_SETTING;				
	WIM_task &= ~WIM_FLASH_TO_COMP;				
	WIM_task |= WIM_ENCODER_OFF;				
	WIM_task |= WIM_MODE_USB;						
	WIM_task &= ~WIM_MODE_UART;						
	WIM_task &= ~WIM_MODE_FLASH_WRITE;		

	
}

void Init_All_Timers (void){
	

	
  WIM_task &= ~WIM_FLASH_TO_COMP;			// Clear flag

	count_overflow = 0;
	Chan1_overflow_0 = 0;
	Chan1_overflow_00 = 0;
	
/*???
	APP_Rx_ptr_in  = 0;
	APP_Rx_ptr_out = 0;
	APP_Rx_length  = 0;
*/
	
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);						// TIM1_CC1
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);				// TIM1_Update
  TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);						// TIM1_CC3

}

void DeInit_All_Timers (void)
{

  TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
  TIM_ITConfig(TIM1, TIM_IT_CC3, DISABLE);

}

void Encoder_on (void)
{

}

void Encoder_off (void)
{

}


//for channel 1 and 3 capture
void TIM1_CC_IRQHandler(void)
{ 
	int64_t counter_chan_1 = 0;			//  CCR1 value of TIM1
	int64_t counter_chan_3 = 0;			//  CCR3 value of TIM1

	int64_t counter_chan_1_from_overflow = 0;			// CCR1 value overflow
	int64_t counter_chan_3_from_overflow = 0;			// CCR3 value overflow

	int flag_overflow = 0;		// Number of CNT register overflow
	int pos = 0;
	char flag_Update_TIM1 = 0;
	uint16_t TIM1_CNT = 0;		

	TIM8_CNT = TIM8->CNT;
	count_overflow_TIM8 = 0;
	
  if((TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) && (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET))
	{
		counter_chan_1 = ((int64_t)(TIM1->CCR1));					// CCR1 reading 
		counter_chan_3 = ((int64_t)(TIM1->CCR3));					// CCR3 reading
		flag_overflow = count_overflow;
		flag_Update_TIM1 = TIM_GetITStatus(TIM1, TIM_IT_Update);
		TIM1_CNT = TIM1->CNT;
	
	
		if ((flag_Update_TIM1 != RESET) && (counter_chan_3 < Counter_PoluPeriod)) pos = 1;
		else if ((((uint64_t)counter_chan_3) > 0xF000) && (TIM1_CNT < 0x8000) && (TIM_GetITStatus(TIM1, TIM_IT_Update) == 0)) pos = -1;
		else pos = 0;
		
		counter_chan_3_from_overflow = (int64_t)((counter_chan_3 + (Counter_Period * (int64_t)(flag_overflow+Chan1_overflow_0+pos))) - counter_0) ; 			//signal period duration calc with overflow channel 3
		
				
		if (counter_chan_3_from_overflow < 0)
		{
			counter_chan_3_from_overflow += Counter_Period;
		}
		if (counter_chan_3_from_overflow > 0x20D0)
		{
			aaa++;
		}

		
		if ((flag_Update_TIM1 != RESET) && (counter_chan_1 < Counter_PoluPeriod)) pos = 1;
		else if ((((uint64_t)counter_chan_1) > 0xF000) && (TIM1_CNT < 0x8000) && (TIM_GetITStatus(TIM1, TIM_IT_Update) == 0)) pos = -1;	
		else pos = 0;
		
		counter_chan_1_from_overflow = (int64_t)((counter_chan_1 + (Counter_Period * (int64_t)(flag_overflow+Chan1_overflow_0+pos))) - counter_0) ; 		//signal period duration calc with overflow, channel 1
		Chan1_overflow_00 = -pos;
		
		
		if (counter_chan_1_from_overflow < 0)
		{
			counter_chan_1_from_overflow += Counter_Period;
		}
		if (counter_chan_1_from_overflow > 0x20D0)
		{
			aaa++;
		}


		if (counter_chan_1_from_overflow > counter_chan_3_from_overflow )
		{
			Processing_TIM1_chan3 (counter_chan_3, counter_chan_3_from_overflow);		
			Chan1_overflow_0 = Chan1_overflow_00;
			Processing_TIM1_chan1 (counter_chan_1, counter_chan_1_from_overflow);
		}
		else
		{			
			Processing_TIM1_chan1 (counter_chan_1, counter_chan_1_from_overflow);		
			flag_overflow = count_overflow;
			
			if ((flag_Update_TIM1 != RESET) && (counter_chan_3 < Counter_PoluPeriod)) pos = 1;
			else if ((((uint64_t)counter_chan_3) > 0xF000) && (TIM1_CNT < 0x8000) && (TIM_GetITStatus(TIM1, TIM_IT_Update) == 0)) pos = -1;
			else pos = 0;

			counter_chan_3_from_overflow = (int64_t)((counter_chan_3 + (Counter_Period * (int64_t)(flag_overflow+Chan1_overflow_0+pos))) - counter_0); //signal period duration calc with overflow, channel 3
			
			
			if (counter_chan_3_from_overflow < 0)
			{
				counter_chan_3_from_overflow += Counter_Period;
				//aaa++;
			}
			if (counter_chan_3_from_overflow > 0x20D0)
			{
				aaa++;
			}


			Processing_TIM1_chan3 (counter_chan_3, counter_chan_3_from_overflow);		
		}
	}
	else if ((~(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)) && (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET))
	{
		counter_chan_3 = ((int64_t)(TIM1->CCR3));					
		flag_overflow = count_overflow;
		flag_Update_TIM1 = TIM_GetITStatus(TIM1, TIM_IT_Update);
		TIM1_CNT = TIM1->CNT;

		if ((flag_Update_TIM1 != RESET) && (counter_chan_3 < Counter_PoluPeriod)) pos = 1;
		else if ((((uint64_t)counter_chan_3) > 0xF000) && (TIM1_CNT < 0x8000) && (TIM_GetITStatus(TIM1, TIM_IT_Update) == 0)) pos = -1;
		else pos = 0;

		counter_chan_3_from_overflow = (int64_t)((counter_chan_3 + (Counter_Period * (int64_t)(flag_overflow+Chan1_overflow_0+pos))) - counter_0); //signal period duration calc with overflow, channel 3
		
		if (counter_chan_3_from_overflow < 0)
		{
			counter_chan_3_from_overflow += Counter_Period;
		}
		if (counter_chan_3_from_overflow > 0x20D0)
		{
			aaa++;
		}


		Processing_TIM1_chan3 (counter_chan_3, counter_chan_3_from_overflow);
	}
	else if ((TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) && (~(TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET)))
	{
		counter_chan_1 = ((int64_t)(TIM1->CCR1));					
		flag_overflow = count_overflow;
		flag_Update_TIM1 = TIM_GetITStatus(TIM1, TIM_IT_Update);
		TIM1_CNT = TIM1->CNT;
		
		if ((flag_Update_TIM1 != RESET) && (counter_chan_1 < Counter_PoluPeriod)) pos = 1;
		else if ((((uint64_t)counter_chan_1) > 0xF000) && (TIM1_CNT < 0x8000) && (TIM_GetITStatus(TIM1, TIM_IT_Update) == 0)) pos = -1;
		else pos = 0;

		counter_chan_1_from_overflow = (int64_t)((counter_chan_1 + (Counter_Period * (int64_t)(flag_overflow+Chan1_overflow_0+pos))) - counter_0) ; 			//signal period duration calc with overflow
		Chan1_overflow_0 = -pos;
		
		
		if (counter_chan_1_from_overflow < 0)
		{
			counter_chan_1_from_overflow += Counter_Period;
		}
		if (counter_chan_1_from_overflow > 0x20D0)
		{
			aaa++;
		}


		Processing_TIM1_chan1 (counter_chan_1, counter_chan_1_from_overflow);
	}		

}

//============================================================================================================================================

//for channel 1 capture
void TIM8_CC_IRQHandler(void)
{

	int64_t counter_chan_3 = 0;										// CCR1 value of TIM1
	int64_t counter_chan_3_from_overflow = 0;			// CCR3 value of TIM1

	int flag_overflow = 0;												// CCR3 value overflow
	int pos = 0;
	char flag_Update_TIM8 = 0;
	uint16_t TIM8_CNT_nov = 0;		

	if ((TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET))
	{
		counter_chan_3 = ((int64_t)(TIM8->CCR2));		//reading CCR2			
		flag_overflow = count_overflow_TIM8;
		flag_Update_TIM8 = TIM_GetITStatus(TIM8, TIM_IT_Update);
		TIM8_CNT_nov = TIM8->CNT;

		if ((flag_Update_TIM8 != RESET) && (counter_chan_3 < Counter_PoluPeriod)) pos = 1;
		else if ((((uint64_t)counter_chan_3) > 0xF000) && (TIM8_CNT_nov < 0x8000) && (TIM_GetITStatus(TIM8, TIM_IT_Update) == 0)) pos = -1;
		
		counter_chan_3_from_overflow = (int64_t)((counter_chan_3 + (Counter_Period * (int64_t)(flag_overflow+Chan2_overflow_0+pos))) - TIM8_CNT) ; 			//signal period duration calc with overflow 
		
		if (counter_chan_3_from_overflow < 0) counter_chan_3_from_overflow += Counter_Period;

		Processing_TIM1_chan3 (counter_chan_3, counter_chan_3_from_overflow);
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);								
	}

}

//Overflow CNT register handler
void TIM8_UP_TIM13_IRQHandler(void) 
{
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);					//reset IRQ
    count_overflow_TIM8++;
		buff_1[ss] = (uint16_t)TIM8->CNT;
		ss++;
		if (ss>=64)	ss=0;
	}
}


void Processing_TIM1_chan1 (int64_t counter_CNT_chan_1, int64_t counter_CNT_chan_1_from_overflow)
{
	uint16_t k;
	
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);		

		counter_1 = ((int64_t) counter_CNT_chan_1);					// Read capture value
		counter_t = counter_CNT_chan_1_from_overflow;
    counter_t_float = (float)(((float) counter_t) *delta_t);		//period duration calc, mcs

		ptr_buff_temp_float = (float *)(buff_temp);
		ptr_buff_temp_float[0] = counter_t_float;					
		Len_buff_temp_char = 4;

    for(k=0; k<Len_buff_temp_char; k++)			// push to FIFO
		{
			if ( !FIFO_IS_FULL( MK_to_Comp_fifo ) )
			{
				FIFO_PUSH( MK_to_Comp_fifo, buff_temp[k] );	
			}
			else
			{
				// If FIFO is overflowed  - turn off Encoder!!
				if((WIM_task & WIM_ENCODER_OFF) == 0) 		
				{
					DeInit_All_Timers ();							
					WIM_task |= WIM_ENCODER_OFF;		
					WIM_task |= WIM_FIFO_FILLED;		
				}
			}
		}		


    WIM_task |= WIM_MK_TO_COMP;		// Flag - data is ready
		counter_0 = counter_1;		// saving current 
    count_overflow = 0;
}

//============================================================================================================================================

void Processing_TIM1_chan3(int64_t counter_CNT_chan_3, int64_t counter_CNT_chan_3_from_overflow)
{
	uint16_t k;
	
    // Clear TIM1 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);					

		counter_1 = ((int64_t)counter_CNT_chan_3);					
		counter_t = counter_CNT_chan_3_from_overflow;
    counter_t_float = (float)(((float) counter_t) *delta_t);		//period duration calc, mcs


		//Prepare data
		ptr_buff_temp_float = (float *)(buff_temp);
		ptr_buff_temp_float[0] = -counter_t_float;					
		Len_buff_temp_char = 4;

    for(k=0; k<Len_buff_temp_char; k++)			// Wtite to FIFO
		{
			if ( !FIFO_IS_FULL( MK_to_Comp_fifo ) )
			{
				FIFO_PUSH( MK_to_Comp_fifo, buff_temp[k] );	
			}
			else
			{
				// If FIFO is overflowed  - turn off Encoder!!
				if((WIM_task & WIM_ENCODER_OFF) == 0) 		
				{
					DeInit_All_Timers ();							
					WIM_task |= WIM_ENCODER_OFF;		
					WIM_task |= WIM_FIFO_FILLED;		
				}
			}
		}		

    WIM_task |= WIM_MK_TO_COMP;		// Flag - data is ready
}

//======================================================================================================================================


void Encoder_proc(void){

	  
	  if(WIM_task & WIM_MK_TO_COMP) //FIFO isn't empty?
	  {
			if(WIM_task & WIM_MODE_UART)
			{
				enc_uart_out(); //!!!USART1_data_output ();
			}
			else if (((WIM_task & WIM_MODE_USB) != 0) || (((WIM_task & WIM_FLASH_TO_COMP) != 0) && ((WIM_task & WIM_MODE_FLASH_WRITE) != 0)))
			{
				if (FIFO_COUNT(MK_to_Comp_fifo) >= 512)
				{
					enc_usb_out();
				}
			}
			else if (((WIM_task & WIM_MODE_FLASH_WRITE) != 0) && ((WIM_task & WIM_FLASH_TO_COMP) == 0))
			{
				if (FIFO_COUNT(MK_to_Comp_fifo) >= 512)
				{
					enc_sd_out();					
				}
			}
		}

		// if fifo is filled - error generate
    if(WIM_task & WIM_FIFO_FILLED)
		{
			uint16_t k, n;

			for (n=0; n<2; n++)
			{
				// Error message
				buff_temp[0] = 0x26;
				buff_temp[1] = 0x45;
				buff_temp[2] = 0x52;
				buff_temp[3] = 0x52;
				buff_temp[4] = 0x4F;
				buff_temp[5] = 0x52;
				buff_temp[6] = 0x2C;
				buff_temp[7] = 0x31;
				buff_temp[8] = 0x0D;
				buff_temp[9] = 0x0A;
				Len_buff_temp_char = 10;

				// Write error FIFO
				for(k=0; k<Len_buff_temp_char; k++)			
				{
					if ( !FIFO_IS_FULL( Error_fifo ) )
					{
						FIFO_PUSH( Error_fifo, buff_temp[k] );	
					}
				}
				
				switch(n)
				{
					case 0 :		//Send to USB
						//OUT_DataTx_USB_Error ();
						for(; FIFO_IS_EMPTY( Error_fifo) != 1 ; ){} 
					break;

					case 1 :		//Send to UART
						for(; FIFO_IS_EMPTY( Error_fifo) != 1 ; ){} 		//USART1_data_output_Error ();
					break;
					default : ;
				}
			}
		  WIM_task &= ~WIM_FIFO_FILLED;			//Reset FIFO flag
	  }

		
    //Castm isn't recieved?
    if(WIM_task & WIM_INPUT_SETTING)
    {
			ask_input_settings();
    }
		
    //Check flash to PC mode out 
    if(WIM_task & WIM_FLASH_TO_COMP)
		{
			enc_sd_read();
		}
		
    //Time has come to erase Flash?
    if(WIM_task & WIM_MODE_FLASH_ERASE)
		{
			
			enc_sd_clear();								
		  WIM_task &= ~WIM_MODE_FLASH_ERASE;
		}
		

}	

void enc_uart_out(void){
	for(; FIFO_IS_EMPTY( Error_fifo) != 1 ; ){}
}

void enc_usb_out(void){
	
	//OUT_DataTx_USB ();
	for(; FIFO_IS_EMPTY( Error_fifo) != 1 ; ){}
}

void enc_sd_out(void){

	for(; FIFO_IS_EMPTY( Error_fifo) != 1 ; ){}
/*
		SDIO_SD_SingleBlock_Rec();
	
		cnt_sector++;
		if(cnt_sector==NUM_REFRESH_SD_INFO){
				cnt_sector = 0;
				SD_Write_Data_Info_Sector0();
		}	
*/
	
}	

void enc_sd_read(void){
	//SDIO_SD_ReadMultiBlocks();
}

void enc_sd_clear(void){
	//SDIO_SD_Erase();		
}

void ask_input_settings(void){
	
	 //if (Setting_Processor() == 1) Parcel_analysis();
	//!!!We should check FIFO Comp_to_MK that fillingby func usbd_cdc_DataOut() inside usbd_cdc_core.c
}

