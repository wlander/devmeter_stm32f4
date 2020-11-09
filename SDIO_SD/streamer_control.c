
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

uint8_t Button_read = 0;
uint8_t Button_catch = 0;
uint8_t Button_tmp = 0;
uint8_t Button_read_cnt = 0;
uint8_t fl_button_on = 0;
uint8_t Button = 0;
uint8_t Reset_Data_Button = 0;
uint8_t Reset_Data_Button_cnt = 0;
uint8_t Reset_Data_Button_Catch = 0;


uint8 wait_sd_ctr_start(){
	
	while(1){
		
		Button_read_cnt = 0;
		Button_read = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		
		if(Button_read==1){
			
				Button_read_cnt++;
				
				for(ii=0;ii<200000;ii++){ii = ii;}		
				Button_read = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
				if(Button_read==1)	Button_read_cnt++;
				
				for(ii=0;ii<200000;ii++){ii = ii;}	
				Button_read = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);	
				if(Button_read==1)	Button_read_cnt++;

		}
		
		if(Button_read_cnt==3){ //if press of start button
			
			//read last data information in sd-card
			SD_Read_Data_Info_Sector0();
			
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}		
			
			break;
		}
		
		Reset_Data_Button_cnt = 0;
		Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		
		if(Reset_Data_Button==1){

				Reset_Data_Button_cnt++;
					
				for(ii=0;ii<200000;ii++){ii = ii;}		
				Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
				if(Reset_Data_Button==1)	Reset_Data_Button_cnt++;
					
				for(ii=0;ii<200000;ii++){ii = ii;}	
				Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);	
				if(Reset_Data_Button==1)	Reset_Data_Button_cnt++;

		}
		
		if(Reset_Data_Button_cnt==3){ //if reset sd card info button
			
			N_block_rec_flash = 0;
			cnt_file_write_flash = 0;

			//reset sd card info
			SD_Write_Data_Info_Sector0();
			
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
			for(ii=0;ii<50000000;ii++){ii = ii;}
			
			break;			
		}
		
	}
	
	Button_read = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
	while (Button_read!=0){	
		for(ii=0;ii<500000;ii++){ii = ii;}
		Button_read = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);		
	}
	
	Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);	
	while (Reset_Data_Button!=0){		
		for(ii=0;ii<500000;ii++){ii = ii;}
		Reset_Data_Button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	}	
	
	fl_button_on = 1;	
	Button_read_cnt = 0;
	Reset_Data_Button_cnt = 0;	
	Button_read = 0;
	Button_catch = 0;	
	
}


