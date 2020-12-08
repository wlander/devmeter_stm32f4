/*******************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "sd_streamer.h"
#include "adc_module_def.h"
#include "conf_init.h"
#include "stm32f4_discovery_sdio_sd.h"
#include  "sd_stream_cntrl.h"
#include "esp_uart.h"

//#include "sdio_sd.h"
//#include "diskio.h"

/*******************************************************************/

uint32_t get_time_dwt(void);
void cycle_buf_time_dwt(void);
void dwt_cnt_run(void);
void Obrab_Conf_byte(void);
void decim_adc_data(uint16_t* ptr_adc_data, uint16_t* ptr_out, uint16_t k_decim, uint16_t n_ch);
void pack_adc_data_2ch(uint16_t* ptr_adc_data, uint16_t* ptr_out);
void unpack_adc_data_2ch(uint8_t* ptr_in, uint16_t* ptr_out);
void pack10_adc_data_2ch(uint16_t* ptr_adc_data, uint16_t* ptr_out);
void unpack10_adc_data_2ch(uint8_t* ptr_in, uint16_t* ptr_out);
void pack8_adc_data_2ch(uint16_t* ptr_adc_data, uint8_t* ptr_out);
void pack16_adc_data_2ch(uint16_t* ptr_adc_data, uint8_t* ptr_out);
//------------ Global -------------------------------------------
uint8_t UART_txMode = 0;
uint8_t UART_Tr_En = 0;
uint8_t Pack_Mode = 0;
uint8_t Pack_Mode_Ch = 0;
uint8_t WindowMode = 0;

uint32_t cnt_globe_cycle = 0;
uint32_t adc_recv_end = 0;
uint32_t adc_recv_f = 0;
uint8_t adc_num_buf = 1;
//---------------------------------------------------------------

//--------------- For ADC -------------------------------------------/
uint16_t ADC_buf1[BUF_ADC_LEN_CH*N_CHANNEL];  
uint16_t ADC_buf2[BUF_ADC_LEN_CH*N_CHANNEL]; 

uint16_t K_Decim = K_DECIM_DEF;
uint16_t* ptr_buf_recv = ADC_buf1;
uint16_t* ptr_buf_obr = ADC_buf2;

uint32_t cnt_recv = 0;
uint32_t cnt_obr = 0;
uint8_t f_adc_complete = 0;
uint8_t cnt_delay_tr = 0;

uint8_t ADC_Fd = SPS_ADC_DEFAULT;

uint16_t KNorm_Ch1  = 2040;
uint16_t KNorm_Ch2  = 2040;
uint16_t KNorm_Ch3  = 0;//2038;

uint16_t Norm_Ch[3]  = {2040,2040,0}; //{2038,2038,2038};
/*******************************************************************/

//--------------- For Send Data ------------------------------------/
uint16_t buf1[BLOCK_SIZE*NUM_BLOCKS_FIFO+32];  
uint16_t buf_uart[BUF_UART_SIZE];//[BUF_ADC_LEN_CH*N_CHANNEL_TX/K_DECIM_DEF+32]; 
uint16_t cnt_uart_send = 0;

uint8_t num_block_fif0_write = 0;
uint8_t num_block_fif0_read = 0;
uint8_t cnt_block_write = 0;
uint8_t cnt_block_read = 0;
uint16_t* block_fifo_ptr = buf1;
uint8_t block_fifo_status = 0;

uint16_t tmp_buf[32]; //[BUF_ADC_LEN_CH*N_CHANNEL_TX/K_DECIM_DEF]; 

uint16_t* ptr_buf_transm = buf1;
//-------------------------------------------------------------------/

//-------------------------------------------------------------------
uint8_t Conf_byte = 0;
uint8_t New_Conf_byte = 0;
uint32_t cnt_rcv_usart1 = 0;

uint32_t cnt_pr_uart = 0;
uint16_t rd_uart = 0;
uint32_t cnt_dma_send_usart = 0;

uint16_t Num_Ch = 2;
uint16_t SizeByte = BUF_ADC_LEN_CH/K_DECIM_DEF*2; //BUF_ADC_LEN_CH*N_CHANNEL/K_DECIM_DEF*2;
//--------------- AD8555 --------------------------------------------
uint8_t ad8555_rcv_par_buf[2];
uint8_t ad8555_flag_rcv_par = 0;
uint8_t ad8555_flag_rcv_par_end = 0;
uint8_t ad8555_cnt_rcv_par = 0;
uint8_t f_ad8555_on = 0;
//-----------------------------------------------------------------------

//------------- SD + SPI ---------------------------------------

uint8_t SD_Mode = 0;
uint8_t SD_rdwr_stat = 0;

uint32_t cnt_sector_wr = 0;
uint32_t cnt_sector_rd = 0;
uint32_t Sector_Num = 3906250;
//void wr_sd_card_adc_data(void);
//void rd_sd_card_adc_data(void);

//DSTATUS DS;
//--------------------------------------------------------------

//-------------- DEBUG ------------------------------------------
#define LEN_DWT_BUF 16
uint8_t f_run_dwt = 0;
uint32_t dwt_cnt_buf[LEN_DWT_BUF] = {0};
uint8_t dwt_buf_c = 0;
uint32_t dwt_last = 0;
uint32_t cnt_dma_uart_run = 0;
uint16_t cnt_test_uart = 0;
uint32_t cnt_wr_sd_card = 0;
uint32_t fault_time_uart = 0;
uint32_t fault_time_sd_wr = 0;
#define DWT_CYCCNT *(volatile uint32_t *) 0xE0001004
#define DWT_CONTROL *(volatile uint32_t *) 0xE0001000
#define SCB_DEMCR *(volatile uint32_t *) 0xE000EDFC

uint32_t cnt1 = 0;
uint32_t cnt2 = 0;
uint32_t cnt3 = 0;

void DMA2_Stream0_IRQHandler( void ){
	uint16_t* ptr_tmp;	
	
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0)){
	   DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
	
	ptr_tmp = ptr_buf_recv;
	ptr_buf_recv = ptr_buf_obr;
	ptr_buf_obr = ptr_tmp;

	if(cnt_block_write<=NUM_BLOCKS_FIFO){
			decim_adc_data(ptr_buf_obr, block_fifo_ptr+num_block_fif0_write*BLOCK_SIZE, K_Decim, Num_Ch);
			cnt_block_write++;
			num_block_fif0_write++;
			if(num_block_fif0_write==NUM_BLOCKS_FIFO) num_block_fif0_write = 0;
	}
	else{
			block_fifo_status = 1;
	}
								
	cnt_recv++;
	f_adc_complete = 1;	
	adc_recv_end++;
	
}


void DMA2_Stream7_IRQHandler( void ){
	                                                                                                       
	cnt_dma_send_usart++;
  NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);  // Clear interrupt
	
}

//---------------------------------------------------------------------

void USART1_IRQHandler(void){
	uint8_t tmp_b;
	tmp_b = USART1->DR;
	
	if(ad8555_flag_rcv_par==1){
	  ad8555_rcv_par_buf[ad8555_cnt_rcv_par] = tmp_b;
		ad8555_cnt_rcv_par++;
		if(ad8555_cnt_rcv_par==2){
		   ad8555_cnt_rcv_par=0;
			 ad8555_flag_rcv_par=0;
			 ad8555_flag_rcv_par_end = 1;
		}
	}
	else{
	  if(tmp_b==RECV_AD8555_PAR) ad8555_flag_rcv_par = 1;
	  else New_Conf_byte = tmp_b;
  }
	
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	cnt_rcv_usart1++;
}

//================ MAIN ==================================================================================================

int main(){
uint32_t i = 0;	
uint8_t St = 0;		

//--------------------------------------------------------------------	
	
/*******************************************************************/
	
//----------- INIT ---------------------------------------------------
	
RCC_init_all();
GPIO_init_all();
SDIO_SD_Init();

ADC_triple_dma_init((uint32_t*)ADC_buf1, (uint32_t*)ADC_buf2, BUF_ADC_LEN_CH*N_CHANNEL, 1);
NVIC_Configuration();		

//SizeByte = 	(BUF_ADC_LEN_CH/K_DECIM_DEF*2*2*8)/10; //BUF_ADC_LEN_CH*Num_Ch/K_DECIM_DEF*2;

#ifdef UNIT_TEST1
	esp_unit_test1();
	//while(1);
#endif	
	
	
#ifdef ESP_EN
	St = init_esp_uart();
#else
	USART1_init((uint32_t*)ptr_buf_transm, SizeByte);
#endif

//USART1_dma_init((uint32_t*)ptr_buf_transm, SizeByte);

__enable_irq();


//Set_ADC_SampleTime(ADC_Fd);		
//ADC_triple_dma_run();		
	
//------------ END INIT ----------------------------------------------
 St = 0;
 
#ifdef ESP_EN
	St = esp_init_udp();
	
	if(St==ESP_UDP_INIT_ERROR){
		//send info to PC about it
		St = St;
	}
	
#endif

#ifdef UNIT_TEST2 

N_block_rec_flash = 0;
cnt_file_write_flash = 1;
	
clear_fifo_esp();
	
 while(1){
	 
 		if((UART_txMode > 0)){	
			
			cntr_sd.cnt_status_write = N_block_rec_flash;
			cntr_sd.cnt_file_sd = cnt_file_write_flash;
			cntr_sd.stat[0] = 3;
			for(i=0; i<SIZE_DATA_CDG; i++) cntr_sd.data[i] = i*N_block_rec_flash;
			
			//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, 16+SIZE_DATA_CDG*2);
			//if(St!=ESP_TR_FIFO_ISFULL) 
			
		  if(cnt_uart_send*SIZE_DATA_CDG>=BUF_UART_SIZE) cnt_uart_send = 0;
/*			
			//esp_uart_msg_tr();
			if(esp_msg_tr_status()!=0){
						esp_uart_msg_tr_continue();
			}
			else{
				St = esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
				
				if(St==1) cnt_uart_send++;
			}
*/
			esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
			//while(esp_uart_msg_tr_continue()!=0) esp_check_rcv_msg();

			N_block_rec_flash++;		
		
		}

		St = esp_check_rcv_msg();		
		esp_uart_msg_tr_continue();
		
		if(St!=0) New_Conf_byte = St; 

		if(St==1)	New_Conf_byte = START_txUART_ADC;
		else if(St==2) New_Conf_byte = STOP_txUART_ADC;
		
		if(New_Conf_byte!=Conf_byte) Obrab_Conf_byte();
		
		for(i=0;i<10000000;i++){i=i;}	
 }
#endif
 
 
 while(1){ //waiting start command after reset device - "start write sd" or "reset data sd"

#ifdef EN_CNTRL_BUTTON	
		St = ask_sd_ctr_btn(); //ask button start or reset
#endif

#ifdef ESP_EN	 
		St = esp_check_rcv_msg();
		if(St!=0) New_Conf_byte = St; 
#endif
	 
		if((New_Conf_byte!=Conf_byte) && (St==0)){ //ask uart0 start or reset
			Obrab_Conf_byte();
			if(Conf_byte == Power_SD_On) St=1;
			else if(Conf_byte == RESET_WR_SD) St=2;	
		}
		
		if(St==2){ //if reset
			N_block_rec_flash = 0;
			cnt_file_write_flash = 0;		
			SD_Write_Data_Info_Sector0(); //write reset data information in sd-card sector0
			fl_start_write_flash = 1;
			break;		
		}	
		else if(St==1){ //if start 				
			SD_Read_Data_Info_Sector0();	//read last data information in sd-card sector0
			fl_start_write_flash = 1;
			break;		
		}
		
	}
 
	St = 0; //reset status control	
	UART_Tr_En = 1;
	
	if(UART_Tr_En){
				
			cntr_sd.cnt_status_write = N_block_rec_flash;
			cntr_sd.cnt_file_sd = cnt_file_write_flash;	
			cntr_sd.stat[0] = 1;		
			if(UART_Tr_En){
				
				#ifdef ESP_EN
					//create task data transmit for esp;
					//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, sizeof(cntr_sd));
				  St = esp_uart_msg_tr_start((uint8_t*)&cntr_sd, 16);
					while(esp_uart_msg_tr_continue()!=0) esp_check_rcv_msg();
				
				#else
					Cycle_UART_tx((uint8_t*)&cntr_sd, sizeof(cntr_sd));
				#endif
			}
	
	}
	
  while(1){
	
			if(f_adc_complete){							
					f_adc_complete = 0;
					cnt_obr++;
					adc_recv_end--;					
			}
		
			if(((UART_txMode > 0)) && (cnt_block_read == 0) && (N_block_rec_flash_tmp == N_block_rec_flash)){	//if starting new file
					fl_start_write_flash = 1;
				
					cntr_sd.cnt_status_write = N_block_rec_flash;
					cntr_sd.cnt_file_sd = cnt_file_write_flash;
					cntr_sd.stat[0] = 2;	
				
					if(UART_Tr_En){
						#ifdef ESP_EN
							//create task data transmit for esp;
							//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, sizeof(cntr_sd));
								esp_uart_msg_tr_start((uint8_t*)&cntr_sd, 16);
								//St = esp_uart_msg_tr_((uint8_t*)&cntr_sd, 16);
						#else
							Cycle_UART_tx((uint8_t*)&cntr_sd, sizeof(cntr_sd));
						#endif					
					}
			}		
			
			if(((UART_txMode > 0)) && (cnt_block_write != 0)){	

					if(cnt_blink==PERIOD_BLINK) GPIO_SetBits(GPIOA, BLINK_CNTRL_PIN);
					
					St = SDIO_SD_SingleBlock_Rec((uint8_t*)(block_fifo_ptr+num_block_fif0_read*BLOCK_SIZE), NUM_SECTORS_SD_WRITE);		
					
					if(cnt_blink==PERIOD_BLINK/2) GPIO_ResetBits(GPIOA, BLINK_CNTRL_PIN);
					
					if(cnt_blink==PERIOD_BLINK) cnt_blink = 0;
					cnt_blink++;
					
					cnt_sector++;
					if(cnt_sector==NUM_REFRESH_SD_INFO){
						cnt_sector = 0;
						SD_Write_Data_Info_Sector0();
					}		
					
					num_block_fif0_read++;
					if(num_block_fif0_read==NUM_BLOCKS_FIFO) num_block_fif0_read = 0;
					cnt_block_write--;
					cnt_block_read++;	
					
					cntr_sd.cnt_status_write = N_block_rec_flash;
					cntr_sd.cnt_file_sd = cnt_file_write_flash;
					cntr_sd.stat[0] = 3;
					
					if(UART_Tr_En){
						if(cnt_uart_send==0){
							for(i=0;i<BUF_UART_SIZE;i++) buf_uart[i] = block_fifo_ptr[i];
						}		
						
						for(i=0; i<SIZE_DATA_CDG; i++) cntr_sd.data[i] = buf_uart[cnt_uart_send*SIZE_DATA_CDG+i];
						
						#ifdef ESP_EN
							//create task data transmit for esp;
							//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, sizeof(cntr_sd));
						  if(esp_msg_tr_status()==0){
								if(cnt_delay_tr==0)	St = esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
								cnt_delay_tr++;
								if(cnt_delay_tr==NUM_DELAY_TR) cnt_delay_tr=0;
							}

						#else
							Cycle_UART_tx((uint8_t*)&cntr_sd, sizeof(cntr_sd));
							cnt_uart_send++;
						#endif	
						

						if(cnt_uart_send*SIZE_DATA_CDG>=BUF_UART_SIZE) cnt_uart_send = 0;
					}	
		}
		else if(((UART_txMode == 0)) && (N_block_rec_flash_tmp != N_block_rec_flash)){ //if stop writing current file
			
				N_block_rec_flash_tmp = N_block_rec_flash;
				SD_Write_Data_Info_Sector0();
				cnt_block_read = 0;
			
				cntr_sd.cnt_status_write = N_block_rec_flash;
				cntr_sd.cnt_file_sd = cnt_file_write_flash;
				cntr_sd.stat[0] = 4;
					
				if(UART_Tr_En){
						#ifdef ESP_EN
							//create task data transmit for esp;
							//St = put_esp_uart_fifo_tr((uint8_t*)&cntr_sd, sizeof(cntr_sd));
							esp_uart_msg_tr_start((uint8_t*)&cntr_sd, sizeof(cntr_sd));
							cnt_delay_tr = 0;
						#else
							Cycle_UART_tx((uint8_t*)&cntr_sd, sizeof(cntr_sd));
						#endif					
				}
			
		}

	#ifdef EN_CNTRL_BUTTON	
		St = ack_cnt_bttn();
	#endif

	#ifdef ESP_EN
//		esp_uart_msg_tr();
		St = esp_check_rcv_msg();		
		esp_uart_msg_tr_continue();

		if(St!=0) New_Conf_byte = St; 
	#endif
		
		if(St==1)	New_Conf_byte = START_txUART_ADC;
		else if(St==2) New_Conf_byte = STOP_txUART_ADC;
		
		if(New_Conf_byte!=Conf_byte){
			Obrab_Conf_byte();
		}
	
		cnt_globe_cycle++;	

		//if(cnt_globe_cycle>1000) 	UART_txMode = 1;
  }
}
 
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

	switch (New_Conf_byte)
  {
//----------------------		
    case(STOP_txUART_ADC) :
			UART_txMode = 0;
			ADC_triple_dma_stop();
      break;
    case (START_txUART_ADC):
			
			if(WindowMode==0)	UART_txMode = 1;
			else UART_txMode = 3; //perevodim okonny rejim fast Fd
		
			adc_recv_end = 0;
			adc_recv_f = 0;
		
			ptr_buf_recv = ADC_buf1;
			ptr_buf_obr = ADC_buf2;
		
	//		ADC_triple_dma_init((uint32_t*)ptr_buf_recv, BUF_ADC_LEN_CH*N_CHANNEL, WindowMode);
			Set_ADC_SampleTime(ADC_Fd);
			ADC_triple_dma_run(); //start ADC
			
      break;
		case (TEST_txUART_cnt):
			UART_txMode = 2;
      break;
		case (RESET_WR_SD):
			cnt_sector_wr = 0;
			break;
		case (Power_SD_On):
			//SD_PowerOn();
			//SD_Stat = disk_initialize(0);
      break;	
		case (Conf_Byte_data_On):
			UART_Tr_En = 1;
			break;
		case (Conf_Byte_data_Off):
			UART_Tr_En = 0;
			break;		
		case (WINDOW_MODE_FAST_FD):
			ADC_triple_dma_stop();
			WindowMode = 1;
			break;		
		case (WINDOW_MODE_STOP):
			WindowMode = 0;
			UART_txMode = 0;
			ADC_Fd = SPS_ADC_DEFAULT;
			break;	
		case (MODE_PACK_EN):
			Pack_Mode = 1;
			break;
		case (MODE_PACK_DIS):
			Pack_Mode = 0;
			break;
		case (MODE_PACK_CH1):
			Pack_Mode_Ch = 0;
			break;
		case (MODE_PACK_CH2):
			Pack_Mode_Ch = 1;
			break;
		case (AD8555_CH_ON):
			f_ad8555_on = 1;
			Norm_Ch[0] = 0;
			break;
		case (AD8555_CH_OFF):
			f_ad8555_on = 0;
		  Norm_Ch[0] = KNorm_Ch1;
			break;		
//----------------------    
		default:
      break;
  }
	
	if((New_Conf_byte>17)&&(New_Conf_byte<26) && (WindowMode==1)){
		  ADC_Fd = New_Conf_byte;
	}
	
	Conf_byte = New_Conf_byte;
}

void dwt_cnt_run(void){
	
	dwt_last = 0;
	SCB_DEMCR |= 0x1000000;
	DWT_CYCCNT = 0;
	DWT_CONTROL |= 1;
	f_run_dwt = 1;	
}

uint32_t get_time_dwt(void){
	uint32_t time_dwt;
	time_dwt = DWT_CYCCNT-dwt_last;
	return time_dwt;
}

void cycle_buf_time_dwt(void){
	uint32_t time_dwt;
	time_dwt = DWT_CYCCNT-dwt_last;
	dwt_last = DWT_CYCCNT;
	dwt_cnt_buf[dwt_buf_c] = time_dwt;	
	dwt_buf_c++;
	if(dwt_buf_c == LEN_DWT_BUF) dwt_buf_c = 0;
	//return time_dwt;
}
