#ifndef __ADC_MODULE_DEF_H
#define __ADC_MODULE_DEF_H

//--------- DEFINE for RECV UART bluetooth conf byte -------------------------------

//--------- DEFINE Conf ADC data (conf byte) ---------------------------------------
#define START_txUART_ADC	0x31 //bluetooth
#define STOP_txUART_ADC		0x30

#define	TEST_txUART_cnt		2

//----------------------------------------------------------------------------------
#define  Power_SD_On				0x33
#define  RESET_WR_SD				0x38
#define  Conf_Byte_data_On 	0x34
#define  Conf_Byte_data_Off 0x35
#define  Conf_Byte_sig_On 	0x36
#define  Conf_Byte_sig_Off 	0x37
//----------------------------------------------------------------------------------

#define START_txUSB_ADC		10 //usb
#define STOP_txUSB_ADC		11

//--------- DEFINE for SET SPS ADC -------------------------------------------------
#define SPS_ADC_1 18
#define SPS_ADC_2 19
#define SPS_ADC_3 20
#define SPS_ADC_4 21
#define SPS_ADC_5 22
#define SPS_ADC_6 23
#define SPS_ADC_7 24
#define SPS_ADC_8 25

#define SPS_ADC_DEFAULT 25
//----------------------------------------------------------------------------------

#define WINDOW_MODE_FAST_FD		26
#define WINDOW_MODE_STOP	27

#define MODE_PACK_EN	28
#define MODE_PACK_DIS	29
#define MODE_PACK_CH1	30
#define MODE_PACK_CH2	31

#define RECV_AD8555_PAR	35
#define AD8555_CH_ON	36
#define AD8555_CH_OFF	37
//=============== PROG CONF ========================================================

//---------- ADC SETTINGS ----------------------------------------------------------
#define BUF_ADC_LEN_CH	6144 //The Size of Buffer Full. BUF_ADC_LEN/2 - this is size buf for double bufferization
#define K_DECIM_DEF			6 //The Size of Buffer Full. BUF_ADC_LEN/2 - this is size buf for double bufferization 
#define N_CHANNEL  			3 
#define N_CHANNEL_TX		2 
//----------------------------------------------------------------------------------
/*
#define ADC_CH1	ADC_Channel_11  //10
#define ADC_CH1_PIN	GPIO_Pin_1
#define ADC_CH1_PORT	GPIOC
*/

#define ADC_CH1	ADC_Channel_1  //10
#define ADC_CH1_PIN	GPIO_Pin_1
#define ADC_CH1_PORT	GPIOA

#define ADC_CH2 ADC_Channel_13  //11
#define ADC_CH2_PIN GPIO_Pin_3
#define ADC_CH2_PORT	GPIOC

/*
#define ADC_CH_AD8555	ADC_Channel_1
#define ADC_CH_AD8555_PIN	GPIO_Pin_1
#define ADC_CH_AD8555_PORT	GPIOA
*/

#define ADC_CH_AD8555	ADC_Channel_11
#define ADC_CH_AD8555_PIN	GPIO_Pin_1
#define ADC_CH_AD8555_PORT	GPIOC

#define  HEADER_LEN			2
#define  HEADER_CODE		0xFEFE

#define  HEADER_LEN2			4
#define  HEADER_CODE2		0xFF0000FF

#define  HEADER_TYPE2		


#define BLOCK_SIZE 						(BUF_ADC_LEN_CH*N_CHANNEL_TX/K_DECIM_DEF)
#define NUM_SECTORS_SD_WRITE 	((BLOCK_SIZE*2)/512)
#define NUM_BLOCKS_FIFO 		10
#define BUF_UART_SIZE				2000 //(int16)
//------------- CODE DEBUG DEF -----------------------------------------------------
//#define SPI_SD_DEBUG
//----------------------------------------------------------------------------------

#define TEST_ESP

//PS: Calc ADC Sampling: ((((HCLK/2)/APB2 Prescaler)/ADC_Prescaler_Div)/(ADC_SampleTime+12))/K_DECIM_DEF = 0,00426829268292682926829268292683 

#endif
