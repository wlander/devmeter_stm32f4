/********************************************************************************
			
  * @file    proc.c
  * @author  Evgeny Suskov. 
  * @version V1.1.1
  * @date    10-11-2020
  * @brief   This file implements the following functions: 
  *           + wait sync command (button or uart) for synhronize several board
  *           + wait command (button or uart) for prepare and start board
  *     
	
  *  COPYRIGHT (C) 2011 - 2021, Evgeny Suskov, Dmitry Kudryashov. 
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License GPLv3+ as published by
  *  the Free Software Foundation.
  * software distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied
*****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "sd_streamer.h"
#include  "sd_stream_cntrl.h"

uint8_t Button_read = 0;
uint8_t Button_catch = 0;
uint8_t Button_tmp = 0;
uint8_t Button_read_cnt = 0;
uint8_t fl_button_on = 0;
uint8_t Button = 0;
uint8_t Button_Sync = 0;
uint8_t Reset_Data_Button = 0;
uint8_t Reset_Data_Button_cnt = 0;
uint8_t Reset_Data_Button_Catch = 0;
uint32_t cnt_blink = 0;
uint16_t cnt_sector = 0;

uint8_t ask_sync_ctr_btn(void){
	
	Button_Sync = GPIO_ReadInputDataBit(GPIOA, BTN_SYNC_PIN);
	return Button_Sync;

}
uint8_t ask_sd_ctr_btn(void){
	
	int ii;
	uint8_t num_btn = 0;
	
		
		Button_read_cnt = 0;
		Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
		
		if(Button_read==1){
			
				Button_read_cnt++;
				
				for(ii=0;ii<200000;ii++){ii = ii;}		
				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
				if(Button_read==1)	Button_read_cnt++;
				
				for(ii=0;ii<200000;ii++){ii = ii;}	
				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);	
				if(Button_read==1)	Button_read_cnt++;

		}
		
		if(Button_read_cnt==3){ //if press of start button

			GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}				
			GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			
			num_btn = 1;
		}
		
		Reset_Data_Button_cnt = 0;
		Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, BTN2_CNTRL_PIN);
		
		if(Reset_Data_Button==1){

				Reset_Data_Button_cnt++;
					
				for(ii=0;ii<200000;ii++){ii = ii;}		
				Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, BTN2_CNTRL_PIN);
				if(Reset_Data_Button==1)	Reset_Data_Button_cnt++;
					
				for(ii=0;ii<200000;ii++){ii = ii;}	
				Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, BTN2_CNTRL_PIN);	
				if(Reset_Data_Button==1)	Reset_Data_Button_cnt++;

		}
		
		if(Reset_Data_Button_cnt==3){ //if reset sd card info button

			GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}			
			GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			
			GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			
			num_btn = 2;			
		}
		
	
	if(num_btn!=0){
		
			Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
			while (Button_read!=0){	
				for(ii=0;ii<500000;ii++){ii = ii;}
				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);		
			}
			
			Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, BTN2_CNTRL_PIN);	
			while (Reset_Data_Button!=0){		
				for(ii=0;ii<500000;ii++){ii = ii;}
				Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, BTN2_CNTRL_PIN);
			}	
				
			Button_read_cnt = 0;
			Reset_Data_Button_cnt = 0;	
			Button_read = 0;
			Button_catch = 0;				
			
	}
	
	return num_btn;
	
}

uint8_t ack_cnt_bttn(void){

	int ii;
	uint8_t status_btn = 0;
	
		Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
		Button_read_cnt = 0;
		
		if(Button_read!=Button_catch){
	
				Button_tmp = Button_read;		
				Button_read_cnt++;
				
				for(ii=0;ii<50000;ii++){ii = ii;}		
				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
				if(Button_read==Button_tmp)	Button_read_cnt++;
				
				for(ii=0;ii<50000;ii++){ii = ii;}	
				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);	
				if(Button_read==Button_tmp)	Button_read_cnt++;
				
				if(Button_read_cnt==3){
					Button_catch = Button_read;
				  if(Button_catch==1) Button = 1;
				}

		}
		
		if(Button==1){
			
				Button = 0;
								
				for(ii=0;ii<20000000;ii++){ii = ii;} //delay

				Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);
				while (Button_read!=0){	
					for(ii=0;ii<500000;ii++){ii = ii;}
					Button_read = GPIO_ReadInputDataBit(GPIOA, BTN1_CNTRL_PIN);		
				}			
				
				if(fl_button_on==1){
					 fl_button_on=0;
					 status_btn = 2; //stop writing to sd	
					 GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
				}
				else{					
						fl_button_on=1;
						status_btn = 1; //start writing to sd	
					  GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
					  cnt_blink = 0;
				}

		}
		
		return status_btn;
	
}
