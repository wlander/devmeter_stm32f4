/********************************************************************************
			
  * @file    proc.c
  * @author  Evgeny Suskov
  * @version V3.0.1
  * @date    19-01-2021
  * @brief   This file implements the following functions: 
  *           + Initialization and Configuration ADC, SD card, UART
  *           + handling function of button and command for prepare and start performing
  *           + handling function of ADC data, prepare for writing SD card and sending of codoramm with UART  
  *           + handling function of Interrupts (DMA ADC, UART) 
  *           + handling function for control codegram 
  *           + functions for decimation, packaging of ADC data,  
  *     
	
  *  COPYRIGHT (C) 2011 - 2021, Evgeny Suskov, Dmitry Kudryashov. 
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License GPLv3+ as published by
  *  the Free Software Foundation.
 
  * software distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied
*****************************************************************************/

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_flash.h"
#include "sd_streamer.h"
#include "adc_module_def.h"
#include "conf_init.h"
#include "stm32f4_discovery_sdio_sd.h"
#include  "sd_stream_cntrl.h"
#include "esp_uart.h"
#include "proc.h"

static control_p cntrl_p;
static adc_data_control adc_data_cntrl;
static data_control data_cntrl;
static debug_control dbg_cntrl;


void DMA2_Stream0_IRQHandler( void ){
	uint16_t* ptr_tmp;	
	
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0)){
	   DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
	
	ptr_tmp = adc_data_cntrl.ptr_buf_recv;
	adc_data_cntrl.ptr_buf_recv = adc_data_cntrl.ptr_buf_obr;
	adc_data_cntrl.ptr_buf_obr = ptr_tmp;
	
if(cntrl_p.txMode){
	
  data_cntrl.cnt_tim_sync = TIM2->CNT;
	if(data_cntrl.cnt_tim<128) data_cntrl.buf_tim[data_cntrl.cnt_tim] = data_cntrl.cnt_tim_sync;
	data_cntrl.cnt_tim++;	
	
	if(data_cntrl.cnt_block_write<=NUM_BLOCKS_FIFO){
			decim_adc_data(adc_data_cntrl.ptr_buf_obr, data_cntrl.block_fifo_ptr+data_cntrl.num_block_fif0_write*BLOCK_SIZE, adc_data_cntrl.K_Decim, adc_data_cntrl.Num_Ch);
			data_cntrl.cnt_block_write++;
			data_cntrl.num_block_fif0_write++;
			if(data_cntrl.num_block_fif0_write==NUM_BLOCKS_FIFO) data_cntrl.num_block_fif0_write = 0;
	}
	else{
			data_cntrl.block_fifo_status = 1;
	}
								
	adc_data_cntrl.cnt_recv++;
	adc_data_cntrl.f_adc_complete = 1;
	
}
	adc_data_cntrl.adc_recv_end++;
	
}


void DMA2_Stream7_IRQHandler( void ){
	                                                                                                       
	data_cntrl.cnt_dma_send_usart++;
  NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);  // Clear interrupt
	
}

//---------------------------------------------------------------------

void USART1_IRQHandler(void){
	uint8_t tmp_b;
	tmp_b = USART1->DR;
	
  cntrl_p.New_Conf_byte = tmp_b;

	
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	cntrl_p.cnt_rcv_usart1++;
}


void init_proc(void){

//--------------------------------------------------------------------	
	adc_data_cntrl.K_Decim = K_DECIM_DEF;
	adc_data_cntrl.Num_Ch = 3;
  adc_data_cntrl.SizeByte = 	(BUF_ADC_LEN_CH/K_DECIM_DEF*2*2*8)/10; //BUF_ADC_LEN_CH*Num_Ch/K_DECIM_DEF*2;		
	adc_data_cntrl.ptr_buf_recv = adc_data_cntrl.ADC_buf1;
	adc_data_cntrl.ptr_buf_obr = adc_data_cntrl.ADC_buf2;
	adc_data_cntrl.ADC_Fd = SPS_ADC_DEFAULT;
	
	data_cntrl.block_fifo_ptr = data_cntrl.buf1;
	
	cntrl_p.En_SD = 1;
//--------------------------------------------------------------------	
	
//----------- INIT ---------------------------------------------------
	//SystemInit();				//init clock distribution 
	
  RCC_init_all();
  GPIO_init_all();
  SDIO_SD_Init();
  init_timer();
  ADC_triple_dma_init((uint32_t*)adc_data_cntrl.ADC_buf1, (uint32_t*)adc_data_cntrl.ADC_buf2, BUF_ADC_LEN_CH*N_CHANNEL, 1);
  NVIC_Configuration();		
//--------------------------------------------------------------------
#ifdef ESP_EN
	St = init_esp_uart();
#else
	USART1_init((uint32_t*)data_cntrl.block_fifo_ptr, adc_data_cntrl.SizeByte);
#endif
//--------------------------------------------------------------------	
__enable_irq();
//--------------------------------------------------------------------	

}


uint8_t wait_sync(){

 GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
	
 while(!ask_sync_ctr_btn()){}	
	
 Set_ADC_SampleTime(adc_data_cntrl.ADC_Fd);
 ADC_triple_dma_run(); //start ADC
 
 GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);	

 return 0;
	 
}


uint8_t wait_command_init(){
 uint8_t St = 0;		

//--------- //waiting start command after reset device - "start write sd" or "reset data sd" ---------------	
 while(1){ 

#ifdef EN_CNTRL_BUTTON	
		St = ask_sd_ctr_btn(); //ask button start or reset
#endif

#ifdef ESP_EN	 
		St = esp_check_rcv_msg();
		if(St!=0) New_Conf_byte = St; 
#endif
	 
		if((cntrl_p.New_Conf_byte!=cntrl_p.Conf_byte) && (St==0)){ //ask uart0 start or reset
			Obrab_Conf_byte();
			if(cntrl_p.Conf_byte == Power_SD_On) St=1;
			else if(cntrl_p.Conf_byte == RESET_WR_SD) St=2;	
		}
		
		if(St==2){ //if reset
			cntrl_sd.N_block_rec_flash = START_SECTOR_FLASH;
			cntrl_sd.cnt_file_write_flash = 0;		
			if(cntrl_p.En_SD) SD_Write_Data_Info_Sector0(); //write reset data information in sd-card sector0
			cntrl_sd.fl_start_write_flash = 1;
			break;		
		}	
		else if(St==1){ //if start 				
			if(cntrl_p.En_SD) SD_Read_Data_Info_Sector0();	//read last data information in sd-card sector0
			cntrl_sd.fl_start_write_flash = 1;
			break;		
		}
		
	}
 
	St = 0; //reset status control	
	cntrl_p.Tr_En = 1;
	
	answer_cntr_sd(1);	

 return 0;	
	
}

void answer_cntr_sd(uint8_t stat){
	
			cntr_sd.cnt_status_write = cntrl_sd.N_block_rec_flash;
			cntr_sd.cnt_file_sd = cntrl_sd.cnt_file_write_flash;
			cntr_sd.cnt_status_write_file = cntrl_sd.N_block_rec_curr_file;
			cntr_sd.stat[0] = stat;		
			if(cntrl_p.Tr_En){
				
				#ifdef ESP_EN
					//create task data transmit for esp;
					//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, sizeof(cntr_sd));
				  St = esp_uart_msg_tr_start((uint8_t*)&cntr_sd, 16);
					//while(esp_uart_msg_tr_continue()!=0) esp_check_rcv_msg();
				
				#else
					Cycle_UART_tx((uint8_t*)&cntr_sd, sizeof(cntr_sd));
				#endif
			}	
			
}

uint8_t proc(){
	
 uint32_t i = 0;	
 uint8_t St = 0;		
 	
			if(adc_data_cntrl.f_adc_complete){							
					adc_data_cntrl.f_adc_complete = 0;
					adc_data_cntrl.cnt_obr++;				
			}
		
			if(((cntrl_p.txMode > 0)) && (data_cntrl.cnt_block_read == 0) && (cntrl_sd.N_block_rec_flash_tmp == cntrl_sd.N_block_rec_flash)){	//if starting new file
					cntrl_sd.fl_start_write_flash = 1;
					cntrl_sd.N_block_rec_curr_file = 0;
					answer_cntr_sd(2);
				
			}		
			
			if(((cntrl_p.txMode > 0)) && (data_cntrl.cnt_block_write != 0)){	

					if(cnt_blink==PERIOD_BLINK) GPIO_SetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
					
					if(data_cntrl.cnt_tim==128){
						data_cntrl.cnt_tim = 0;
						if(cntrl_p.En_SD) SDIO_SD_TIM_Rec((uint8_t*)data_cntrl.buf_tim);
					}
				  
					if(cntrl_p.En_SD) St = SDIO_SD_SingleBlock_Rec((uint8_t*)(data_cntrl.block_fifo_ptr+data_cntrl.num_block_fif0_read*BLOCK_SIZE), NUM_SECTORS_SD_WRITE);		
					
					if(cnt_blink==PERIOD_BLINK/2) GPIO_ResetBits(BLINK_CNTRL_PORT, BLINK_CNTRL_PIN);
					
					if(cnt_blink==PERIOD_BLINK) cnt_blink = 0;
					cnt_blink++;
					
					cnt_sector++;
					if(cnt_sector==NUM_REFRESH_SD_INFO){
						cnt_sector = 0;
						if(cntrl_p.En_SD) SD_Write_Data_Info_Sector0();
					}		
					
					data_cntrl.num_block_fif0_read++;
					if(data_cntrl.num_block_fif0_read==NUM_BLOCKS_FIFO) data_cntrl.num_block_fif0_read = 0;
					data_cntrl.cnt_block_write--;
					data_cntrl.cnt_block_read++;	
					
					if(cntrl_p.Tr_En){
						if(data_cntrl.cnt_uart_send==0){
							for(i=0;i<BUF_UART_SIZE;i++) data_cntrl.buf_uart[i] = data_cntrl.block_fifo_ptr[i];
						}		
						
						for(i=0; i<SIZE_DATA_CDG; i++) cntr_sd.data[i] = data_cntrl.buf_uart[data_cntrl.cnt_uart_send*SIZE_DATA_CDG+i];
						
						#ifdef ESP_EN
							if(get_fl_rcv_data_stat() && (!esp_msg_tr_status())){
									reset_fl_rcv_data_stat();
									esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
							}
						#else
							answer_cntr_sd(3);
							data_cntrl.cnt_uart_send++;
						#endif	
						

						if(data_cntrl.cnt_uart_send*SIZE_DATA_CDG>=BUF_UART_SIZE) data_cntrl.cnt_uart_send = 0;
					}	
		}
		else if(((cntrl_p.txMode == 0)) && (cntrl_sd.N_block_rec_flash_tmp != cntrl_sd.N_block_rec_flash)){ //if stop writing current file
			
				cntrl_sd.N_block_rec_flash_tmp = cntrl_sd.N_block_rec_flash;
				if(cntrl_p.En_SD) SD_Write_Data_Info_Sector0();
				data_cntrl.cnt_block_read = 0;
			
				answer_cntr_sd(4);
					
				#ifdef ESP_EN
						cnt_delay_tr = 0;
				#endif
			
		}

	#ifdef EN_CNTRL_BUTTON	
		St = ack_cnt_bttn();
	#endif

	#ifdef ESP_EN
//		esp_uart_msg_tr();
		St = esp_check_rcv_msg();		
		esp_uart_msg_tr_continue();

		if(St!=0) cntrl_p.New_Conf_byte = St; 
	#endif
		
		if(St==1)	cntrl_p.New_Conf_byte = START_txUART_ADC;
		else if(St==2) cntrl_p.New_Conf_byte = STOP_txUART_ADC;
		
		if(cntrl_p.New_Conf_byte!=cntrl_p.Conf_byte){
			Obrab_Conf_byte();
		}
	
		cntrl_p.cnt_globe_cycle++;	

//-------------------------------------------------------------------------------	
	 return 0;
}//end main
 
/*******************************************************************/

void decim_adc_data(uint16_t* ptr_adc_data, uint16_t* ptr_out, uint16_t k_decim, uint16_t n_ch){
	uint32_t i,j;
	
	for(i=0;i<BUF_ADC_LEN_CH/k_decim;i++){
		for(j=0;j<n_ch;j++){
			ptr_out[i*n_ch+j] = ptr_adc_data[i*N_CHANNEL*k_decim+j];//-Norm_Ch[j];
		}
	}
}

void pack_adc_data_2ch(uint16_t* ptr_adc_data, uint16_t* ptr_out){
	uint32_t i;
	uint8_t* ptr_pack;
	uint16_t sample;
	uint8_t* ptr_s = (uint8_t*)&sample;
	
	ptr_pack = (uint8_t*)ptr_out;
	
	for(i=0;i<(BUF_ADC_LEN_CH/K_DECIM_DEF*2)/8;i++){	
		
		sample = (ptr_adc_data[i*8]>>1);	 //1---
	//--- 1	
		ptr_pack[i*11] = ptr_s[0];				//8	
		ptr_pack[i*11+1] = (ptr_s[1]<<5); //3 (free 5)
	//---	2
		sample = (ptr_adc_data[i*8+1]>>1); //2---
		ptr_pack[i*11+1] = ptr_pack[i*11+1] | (ptr_s[0]&0x1F); //5 0b00011111
		sample = (sample>>5);
		ptr_pack[i*11+2] = (ptr_s[0]<<2); //6
	//---	3
		sample = (ptr_adc_data[i*8+2]>>1); //3---
		ptr_pack[i*11+2] = ptr_pack[i*11+2] | (ptr_s[0]&0x3); //2 0b00000011
		sample = (sample>>2);
		ptr_pack[i*11+3] = ptr_s[0]; //8	
		ptr_pack[i*11+4] = ((ptr_s[1]&0x1)<<7); //1	
	//---	4
		sample = (ptr_adc_data[i*8+3]>>1); //4--- 
		ptr_pack[i*11+4] = ptr_pack[i*11+4] | (ptr_s[0]&0x7F); //7 0b01111111
		sample = (sample<<1);
		ptr_pack[i*11+5] = (ptr_s[1]<<4); //4
	//--- 5
		sample = (ptr_adc_data[i*8+4]>>1); //5--- 
		ptr_pack[i*11+5] = ptr_pack[i*11+5] | (ptr_s[0]&0xF); //4 0b00001111
		sample = (sample>>4);
		ptr_pack[i*11+6] = (ptr_s[0]<<1); //7
	//--- 6
		sample = (ptr_adc_data[i*8+5]>>1);
		ptr_pack[i*11+6] = ptr_pack[i*11+6] | (ptr_s[0]&0x1); //1
		sample = (sample>>1);
		ptr_pack[i*11+7] = ptr_s[0];	//8
		ptr_pack[i*11+8] = (ptr_s[1]&0x3); //2
	//--- 7
		sample = (ptr_adc_data[i*8+6]>>1);
		ptr_pack[i*11+8] = ptr_pack[i*11+8] | (ptr_s[0]&0x3F); //6 //0b00111111
		sample = (sample>>6);
		ptr_pack[i*11+9] = (ptr_s[0]<<3);
	//--- 8
		sample = (ptr_adc_data[i*8+7]>>1);
		ptr_pack[i*11+9] = ptr_pack[i*11+9] | (ptr_s[0]&0x7);
		sample = (sample>>3);
		ptr_pack[i*11+10] = ptr_s[0];

	}	
}


void unpack_adc_data_2ch(uint8_t* ptr_in, uint16_t* ptr_out){
	uint32_t i;
	uint8_t* ptr_unpack;
	uint8_t sample;
	
	ptr_unpack = (uint8_t*)ptr_out;
	
	for(i=0;i<(BUF_ADC_LEN_CH/K_DECIM_DEF*2)/8;i++){	
		
	//--- 1			
		sample = ptr_in[i*11];	 //1---
		ptr_unpack[i*16] = sample; //8
		sample = ptr_in[i*11+1];	 //2---
		ptr_unpack[i*16+1] = (sample>>5); //3
	//--- 2	
		ptr_unpack[i*16+2] = (sample&0x1F); //5
		sample = ptr_in[i*11+2];	 //3---
		ptr_unpack[i*16+2] = ptr_unpack[i*16+2] | ((sample<<3)&0xE0); //3
		ptr_unpack[i*16+3] = (sample&0xE0); //3
	//--- 3	
		ptr_unpack[i*16+4] = (sample&0x3); //2
		sample = ptr_in[i*11+3];	 //4---
		ptr_unpack[i*16+4] = ptr_unpack[i*16+4] | ((sample<<2)&0xFC); //6
		ptr_unpack[i*16+5] = (sample>>6); //2
		sample = ptr_in[i*11+4];	 //5---
		ptr_unpack[i*16+5] = ptr_unpack[i*16+5] | ((sample>>5)&0x4); //1
	//--- 4
		ptr_unpack[i*16+6] = (sample&0x7F); //7
		sample = ptr_in[i*11+5]; //6---
		ptr_unpack[i*16+6] = ptr_unpack[i*16+6] | (sample&0x80); //1
		ptr_unpack[i*16+7] = ((sample>>4)&0x7); //3
	//--- 5
		ptr_unpack[i*16+8] = (sample&0xF); //4
		sample = ptr_in[i*11+6]; //7---
		ptr_unpack[i*16+8] = ptr_unpack[i*16+8] | ((sample>>1)<<4); //4
		ptr_unpack[i*16+9] = (sample>>5); //3
	//--- 6
		ptr_unpack[i*16+10] = (sample&0x1); //1
		sample = ptr_in[i*11+7]; //8---
		ptr_unpack[i*16+10] = ptr_unpack[i*16+10] | (sample<<1); //7
		ptr_unpack[i*16+11] = (sample>>7); //1
		sample = ptr_in[i*11+8]; //9---
		ptr_unpack[i*16+11] = ptr_unpack[i*16+11] | ((sample>>6)<<1); //2
	//--- 7
		ptr_unpack[i*16+12] = (sample&0x3F); //6
		sample = ptr_in[i*11+9]; //10---
		ptr_unpack[i*16+12] = ptr_unpack[i*16+12] | ((sample<<3)&0xC0); //2
		ptr_unpack[i*16+13] = (sample>>5); //3
	//--- 8
		ptr_unpack[i*16+14] = (sample&0x7); //3
		sample = ptr_in[i*11+10]; //11---
		ptr_unpack[i*16+14] = ptr_unpack[i*16+14] | (sample<<3); //5
		ptr_unpack[i*16+15] = (sample>>5); //3
		
	}	
/*	
	for(i=0;i<(BUF_ADC_LEN_CH/K_DECIM_DEF*2)/8;i++){
		ptr_out[i] = (ptr_out[i]<<1);
	}	
*/	
}


void pack10_adc_data_2ch(uint16_t* ptr_adc_data, uint16_t* ptr_out){
	uint32_t i;
	uint8_t* ptr_pack;
	uint16_t sample;
	uint8_t* ptr_s = (uint8_t*)&sample;
	
	ptr_pack = (uint8_t*)ptr_out;
	
	for(i=0;i<(BUF_ADC_LEN_CH/K_DECIM_DEF*2)/4;i++){	
		
		sample = (ptr_adc_data[i*4]>>2);	 //1---
		sample = (sample<<6);
		ptr_pack[i*5] = ptr_s[1];				
		ptr_pack[i*5+1] = ptr_s[2]; //2
		
		sample = (ptr_adc_data[i*4+1]>>2);	 //2---		
		sample = (sample<<4);
		ptr_pack[i*5+1] = (ptr_pack[i*5+1] | ptr_s[1]);	//6	
		ptr_pack[i*5+2] = ptr_s[0];
		
		sample = (ptr_adc_data[i*4+2]>>2);	 //3---		
		sample = (sample<<2);
		ptr_pack[i*5+2] = (ptr_pack[i*5+2] | ptr_s[1]);
		ptr_pack[i*5+3] = ptr_s[0];
		
		sample = (ptr_adc_data[i*4+3]>>2);	 //4---		
		ptr_pack[i*5+3] = (ptr_pack[i*5+3] | ptr_s[1]);
		ptr_pack[i*5+4] = ptr_s[0];
		
	}	
}

void pack8_adc_data_2ch(uint16_t* ptr_adc_data, uint8_t* ptr_out){
	uint32_t i;
	uint16_t sample;
	uint8_t* ptr_s = (uint8_t*)&sample;
	
	for(i=0;i<BUF_ADC_LEN_CH/K_DECIM_DEF;i++){	
		
		sample = (ptr_adc_data[i*2]<<5); //delete one bit last and three bit first
		if(ptr_adc_data[i*2]>2047) ptr_s[1] = 0xFF; 
		ptr_out[i*2] = ptr_s[1];		//1		
		sample = (ptr_adc_data[i*2+1]<<6); //delete one bit last and three bit first
		if(ptr_adc_data[i*2+1]>1023) ptr_s[1] = 0xFF; 
		ptr_out[i*2+1] = ptr_s[1]; //2		
		
	}	
}

void pack16_adc_data_2ch(uint16_t* ptr_adc_data, uint8_t* ptr_out){
	uint32_t i;
	uint16_t ish = 0;
	uint16_t sample1;
	uint16_t sample2;
	uint16_t sample_out;	
	uint8_t* ptr_s = (uint8_t*)&sample_out;

	
	for(i=0;i<BUF_ADC_LEN_CH/K_DECIM_DEF;i++){	
			
//		ptr_adc_data[i*2+0] = ish;
		
		sample1 = (ptr_adc_data[i*2+0]>>2); //delete two first bit channel 1
		sample2 = (ptr_adc_data[i*2+1]<<5); //delete one last bit channel 2
			
		if(ptr_adc_data[i*2+0]>2047) sample2 = 0xFFFF;
		
		sample2&=0xFC00; 	//delete three first bit channel 2 the sample result is three to eleven if biger eleven bits number then sample is 4096 
		
		sample_out = (sample2 | sample1);
		
		ptr_out[i*2] = ptr_s[0];		//1		
		ptr_out[i*2+1] = ptr_s[1];
		
		ish++;
		
	}	
	
}

void unpack10_adc_data_2ch(uint8_t* ptr_in, uint16_t* ptr_out){
	uint32_t i;
	
	uint32_t sd;
	uint32_t sd1;
	uint8_t* ptr_sd = (uint8_t*)&sd;
	uint8_t* ptr_s = (uint8_t*)&sd1;
	uint8_t* ptr_unpack;
	
	ptr_unpack = (uint8_t*)ptr_out;
	
	for(i=0;i<(BUF_ADC_LEN_CH/K_DECIM_DEF*2)/5;i++){	
		
		ptr_sd[3] = ptr_in[i*5];
		ptr_sd[2] = ptr_in[i*5+1];
		ptr_sd[1] = ptr_in[i*5+2];
		ptr_sd[0] = ptr_in[i*5+3];
	
		sd1 = (sd>>22);
		ptr_unpack[i*8] = ptr_s[0];
		ptr_unpack[i*8+1] = ptr_s[1];
		
		sd = (sd<<10);
		sd1 = (sd>>22);
		ptr_unpack[i*8+2] = ptr_s[0];
		ptr_unpack[i*8+3] = ptr_s[1];		
		
		sd = (sd<<10);
		sd1 = (sd>>22);
		ptr_unpack[i*8+4] = ptr_s[0];
		ptr_unpack[i*8+5] = ptr_s[1];	
	
		ptr_unpack[i*8+7] = (ptr_in[i*5+3]&0x3);	
		ptr_unpack[i*8+6] = ptr_in[i*5+4];		
		
	}	

}

void exl_header_def(uint8_t* d_src, uint32_t size){
	uint32_t i;
	
	for(i=0;i<size;i++) if(d_src[i]==0xFE) d_src[i] = 0xFC;
		
}


void Obrab_Conf_byte(void){

	switch (cntrl_p.New_Conf_byte)
  {
//----------------------		
    case(STOP_txUART_ADC) :
			cntrl_p.txMode = 0;
//			ADC_triple_dma_stop();
      break;
    case (START_txUART_ADC):
			
			if(cntrl_p.WindowMode==0)	cntrl_p.txMode = 1;
			else cntrl_p.txMode = 3; //perevodim okonny rejim fast Fd
		
			adc_data_cntrl.adc_recv_f = 0;
		
			adc_data_cntrl.ptr_buf_recv = adc_data_cntrl.ADC_buf1;
			adc_data_cntrl.ptr_buf_obr = adc_data_cntrl.ADC_buf2;
		
	//		ADC_triple_dma_init((uint32_t*)ptr_buf_recv, BUF_ADC_LEN_CH*N_CHANNEL, WindowMode);
//			Set_ADC_SampleTime(ADC_Fd);
//			ADC_triple_dma_run(); //start ADC
			
      break;
		case (TEST_txUART_cnt):
			cntrl_p.txMode = 2;
      break;
		case (RESET_WR_SD):
			break;
		case (Power_SD_On):
			answer_cntr_sd(1);
			//SD_PowerOn();
			//SD_Stat = disk_initialize(0);
      break;	
		case (Conf_Byte_data_On):
			cntrl_p.Tr_En = 1;
			break;
		case (Conf_Byte_data_Off):
			cntrl_p.Tr_En = 0;
			break;		
		case (WINDOW_MODE_FAST_FD):
			ADC_triple_dma_stop();
			cntrl_p.WindowMode = 1;
			break;		
		case (WINDOW_MODE_STOP):
			cntrl_p.WindowMode = 0;
			cntrl_p.txMode = 0;
			adc_data_cntrl.ADC_Fd = SPS_ADC_DEFAULT;
			break;	
		case (MODE_PACK_EN):
			cntrl_p.Pack_Mode = 1;
			break;
		case (MODE_PACK_DIS):
			cntrl_p.Pack_Mode = 0;
			break;
		case (MODE_PACK_CH1):
			cntrl_p.Pack_Mode_Ch = 0;
			break;
		case (MODE_PACK_CH2):
			cntrl_p.Pack_Mode_Ch = 1;
			break;			
		case (SD_Rec_Off):
			cntrl_p.En_SD = 0;
			break;	
		case (SD_Rec_On):
			cntrl_p.En_SD = 1;
			break;	
		
//----------------------    
		default:
      break;
  }
	
	if((cntrl_p.New_Conf_byte>17)&&(cntrl_p.New_Conf_byte<26) && (cntrl_p.WindowMode==1)){
		  adc_data_cntrl.ADC_Fd = cntrl_p.New_Conf_byte;
	}
	
	cntrl_p.Conf_byte = cntrl_p.New_Conf_byte;
}

void dwt_cnt_run(void){
	
	dbg_cntrl.dwt_last = 0;
	SCB_DEMCR |= 0x1000000;
	DWT_CYCCNT = 0;
	DWT_CONTROL |= 1;
	dbg_cntrl.f_run_dwt = 1;	
}

uint32_t get_time_dwt(void){
	uint32_t time_dwt;
	time_dwt = DWT_CYCCNT-dbg_cntrl.dwt_last;
	return time_dwt;
}

void cycle_buf_time_dwt(void){
	uint32_t time_dwt;
	time_dwt = DWT_CYCCNT-dbg_cntrl.dwt_last;
	dbg_cntrl.dwt_last = DWT_CYCCNT;
	dbg_cntrl.dwt_cnt_buf[dbg_cntrl.dwt_buf_c] = time_dwt;	
	dbg_cntrl.dwt_buf_c++;
	if(dbg_cntrl.dwt_buf_c == LEN_DWT_BUF) dbg_cntrl.dwt_buf_c = 0;
	//return time_dwt;
}

void unit_tests(void){

	
#ifdef UNIT_TEST1
	esp_unit_test1();
	//while(1);
#endif	
	
	
#ifdef ESP_EN
	St = init_esp_uart();
#else
	USART1_init((uint32_t*)data_cntrl.buf1, adc_data_cntrl.SizeByte);
#endif
	

}

void unit_test2(void){
	int i = 0;
	int St = 0;
	
cntrl_sd.N_block_rec_flash = START_SECTOR_FLASH;
cntrl_sd.cnt_file_write_flash = 1;
	
clear_fifo_esp();
	
 while(1){
	 
 		if((cntrl_p.txMode > 0)){	
			
			cntr_sd.cnt_status_write = cntrl_sd.N_block_rec_flash;
			cntr_sd.cnt_file_sd = cntrl_sd.cnt_file_write_flash;
			cntr_sd.stat[0] = 3;
			for(i=0; i<SIZE_DATA_CDG; i++) cntr_sd.data[i] = i*cntrl_sd.N_block_rec_flash;
						
		  if(data_cntrl.cnt_uart_send*SIZE_DATA_CDG>=BUF_UART_SIZE) data_cntrl.cnt_uart_send = 0;


			if(get_fl_rcv_data_stat() && (!esp_msg_tr_status())){
					reset_fl_rcv_data_stat();
					esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
			}


			cntrl_sd.N_block_rec_flash++;		
		
		}

		St = esp_check_rcv_msg();		
		esp_uart_msg_tr_continue();
		
		if(St!=0){
			cntrl_p.New_Conf_byte = St; 
		}
			
		if(St==1)	cntrl_p.New_Conf_byte = START_txUART_ADC;
		else if(St==2) cntrl_p.New_Conf_byte = STOP_txUART_ADC;
		
		if(St==ASK_OK){
			cntr_sd.cnt_status_write = cntrl_sd.N_block_rec_flash;
			cntr_sd.cnt_file_sd = cntrl_sd.cnt_file_write_flash;
			cntr_sd.stat[0] = 5;
			esp_uart_msg_tr_start((uint8_t*)&cntr_sd, 16);
		}
		
		if(cntrl_p.New_Conf_byte!=cntrl_p.Conf_byte) Obrab_Conf_byte();
		
		for(i=0;i<10000000;i++){i=i;}	
 }
 
}//end unit_test2

