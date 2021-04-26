/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PROC_H
#define __PROC_H


/*******************************************************************/


typedef struct
{	
	uint8_t txMode;										//mode of data trasmit
	uint8_t Tr_En;  									//permission of data trasmit
	uint8_t Pack_Mode; 								//flag for pack mode of data
	uint8_t Pack_Mode_Ch;							//type for pack mode of data
	uint8_t WindowMode; 							//type of capture mode

	uint8_t Conf_byte;								//received configuration byte from UART
  uint8_t New_Conf_byte;
  uint32_t cnt_rcv_usart1;
	
	uint32_t cnt_globe_cycle;					//global processing counter
	
} control_p;


typedef struct
{	

  uint32_t adc_recv_end;
  uint32_t adc_recv_f;	
	uint16_t K_Decim;
	uint16_t* ptr_buf_recv; 
	uint16_t* ptr_buf_obr;  
  uint32_t cnt_recv;
  uint32_t cnt_obr;
  uint8_t f_adc_complete;	
  uint16_t Num_Ch;
  uint16_t SizeByte;
  uint8_t ADC_Fd;	
//--------------- For ADC -------------------------------------------/
	uint16_t ADC_buf1[BUF_ADC_LEN_CH*N_CHANNEL];  
	uint16_t ADC_buf2[BUF_ADC_LEN_CH*N_CHANNEL]; 	
	
} adc_data_control;


typedef struct
{	
	
  uint16_t cnt_uart_send;
  uint32_t cnt_dma_send_usart;
  uint8_t num_block_fif0_write;
  uint8_t num_block_fif0_read;
  uint8_t cnt_block_write;
  uint8_t cnt_block_read;
  uint16_t* block_fifo_ptr;
  uint8_t block_fifo_status;	
  uint16_t cnt_tim;		
  uint32_t cnt_tim_sync;

	
//--------------- For Send Data ------------------------------------/
  uint16_t buf1[BLOCK_SIZE*NUM_BLOCKS_FIFO+32]; //processed data of ADC 
  uint16_t buf_uart[BUF_UART_SIZE]; 						//data for transmition
  uint32_t buf_tim[128];												//processed data of timer 
	
} data_control;


#define LEN_DWT_BUF 16
#define DWT_CYCCNT *(volatile uint32_t *) 0xE0001004
#define DWT_CONTROL *(volatile uint32_t *) 0xE0001000
#define SCB_DEMCR *(volatile uint32_t *) 0xE000EDFC
	
typedef struct
{	

  uint8_t f_run_dwt;
  uint8_t dwt_buf_c;
  uint32_t dwt_last;
  uint32_t cnt_dma_uart_run;
  uint16_t cnt_test_uart;
  uint32_t cnt_wr_sd_card;
  uint32_t fault_time_uart;
  uint32_t fault_time_sd_wr;
  uint32_t cnt2;
  uint32_t cnt3;	
  
	uint32_t dwt_cnt_buf[LEN_DWT_BUF];
	
} debug_control;

void init_proc(void);
uint8_t wait_sync(void);
uint8_t wait_command_init(void);
uint8_t proc(void);
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
void answer_cntr_sd(uint8_t);

extern const uint32_t Start_sector_flash;

void unit_test2(void);


#endif
