//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "misc.h"
#include "adc_module_def.h"
#include "conf_init.h"
#include "stm32f4_discovery.h"
//---------------------------------------------------------------
DMA_InitTypeDef DMA_InitStructure;
DMA_InitTypeDef DMA_USART_InitStructure;
//======================= RCC INIT =====================================================================

void RCC_init_all(void){
	
  /* Enable the GPIOC Clock & ADC1 Periph Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE); //|  RCC_APB2Periph_SDIO
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

}
//=======================================================================================================

//======================= GPIO INIT =====================================================================
void GPIO_init_all(void){
   GPIO_InitTypeDef      GPIO_InitStructure;
	 GPIO_InitTypeDef      USART_GPIO_InitStructure;	
	
   GPIO_InitStructure.GPIO_Pin = ADC_CH1_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(ADC_CH1_PORT, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = ADC_CH2_PIN;
	 GPIO_Init(ADC_CH2_PORT, &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Pin = ADC_CH_AD8555_PIN;
	 GPIO_Init(ADC_CH_AD8555_PORT, &GPIO_InitStructure);
	
	 GPIO_StructInit(&USART_GPIO_InitStructure);
	
//-------- Configure PB.06 as UART TX --------------------------------------	
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure );

//-------- Configure PB.07 as UART RX --------------------------------------
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_Init(GPIOB, &GPIO_InitStructure );		
	 
	 GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	 GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); 
	 
//Led Init	 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_1);	 
	    
}

void NVIC_Configuration(void){
  NVIC_InitTypeDef NVIC_InitStructure;	 

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); //enable dma interrapt for ADC
	
  /* Enable the USART1 Interrupt */
/*   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
*/	
	 //NVIC_DisableIRQ(DMA2_Stream7_IRQn);	//disable interrapt for usart tx

//for SD Card	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  	
	
}

void ADC_triple_dma_init(uint32_t* ptr_adc_data1, uint32_t* ptr_adc_data2, uint32_t buf_size, uint8_t en_pp){
	
   ADC_InitTypeDef ADC_InitStructure;
   ADC_CommonInitTypeDef ADC_CommonInitStructure;

   // Deinitializes the ADC peripheral registers
   ADC_DeInit();
	 
	 DMA_StructInit(&DMA_InitStructure);
   // DMA2 Stream0 channel0 configuration **************************************
   DMA_InitStructure.DMA_Channel = DMA_Channel_0;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ptr_adc_data1;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   DMA_InitStructure.DMA_BufferSize = buf_size;	
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	 
	 DMA_DoubleBufferModeConfig(DMA2_Stream0, (uint32_t)ptr_adc_data2, DMA_Memory_0);
	 DMA_DoubleBufferModeCmd(DMA2_Stream0, ENABLE);
	 
   DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  // DMA2_Stream0 enable
   DMA_Cmd(DMA2_Stream0, ENABLE);
   
	 ADC_CommonStructInit(&ADC_CommonInitStructure);
   // ADC Common configuration -- ADC_CCR Register
   ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult; //ADC_TripleMode_Interl; 
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
   ADC_CommonInit(&ADC_CommonInitStructure);
	 
   ADC_StructInit(&ADC_InitStructure);
   // ADC1 regular channel configuration -- ADC_CR1, ADC_CR2, ADC_SQR1 Register
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ScanConvMode = DISABLE;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	 ADC_InitStructure.ADC_NbrOfConversion = 1;
			 
	 // ADC1 regular channel6 configuration -- ADCx->SMPR1,SMPR2 et ADCx->SQR1,SQR2,SQR3
   ADC_Init(ADC1, &ADC_InitStructure);     
   ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_480Cycles);		 
	 
	 // ADC2 regular channel13 configuration -- ADCx->SMPR1,SMPR2 et ADCx->SQR1,SQR2,SQR3	 
   ADC_Init(ADC2, &ADC_InitStructure); 
   ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_480Cycles);
       
	 // ADC3 regular channell0 configuration -- ADCx->SMPR1,SMPR2 et ADCx->SQR1,SQR2,SQR3
	 ADC_Init(ADC3 , &ADC_InitStructure);
   ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_480Cycles);
 
   // Enable DMA request after last transfer (Multi-ADC mode)  */
   ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
 
 	    // Enable DMA2 Stream0 Transfer complete interrupt 
//	 if(en_pp) DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE); 
/*	 else*/ DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE); 
	 
   ADC_Cmd(ADC1, ENABLE);
   ADC_Cmd(ADC2, ENABLE);
   ADC_Cmd(ADC3, ENABLE);	 	
	 	 
 // Enable ADC1 DMA since ADC1 is the Master
   ADC_DMACmd(ADC1, ENABLE); 	
}
 
void Set_ADC_SampleTime(uint8_t num){

	switch (num){
		case (SPS_ADC_1):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_3Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_3Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_3Cycles);	
      break;
//----------------------
		case (SPS_ADC_2):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_15Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_15Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_15Cycles);	
      break;
		case (SPS_ADC_3):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_28Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_28Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_28Cycles);	
      break;
		case (SPS_ADC_4):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_56Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_56Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_56Cycles);	
      break;
		case (SPS_ADC_5):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_84Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_84Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_84Cycles);	
      break;
		case (SPS_ADC_6):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_112Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_112Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_112Cycles);	
      break;
		case (SPS_ADC_7):
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_144Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_144Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_144Cycles);	
      break;
		default:
			ADC_RegularChannelConfig(ADC1, ADC_CH1, 1, ADC_SampleTime_480Cycles);		
			ADC_RegularChannelConfig(ADC2, ADC_CH2, 1, ADC_SampleTime_480Cycles);	
			ADC_RegularChannelConfig(ADC3, ADC_CH_AD8555, 1, ADC_SampleTime_480Cycles);			
	}
	
} 

void ADC_triple_dma_run(void){
	
//---------------------------------------------------------
  /* Start ADC1 Software Conversion */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);	
	ADC_SoftwareStartConv(ADC1);
	
}

void ADC_triple_dma_stop(void){
	
//---------------------------------------------------------
  /* Start ADC1 Software Conversion */ 
	  ADC_DMACmd(ADC1, DISABLE);
		ADC_Cmd(ADC1, DISABLE);
	  ADC_Cmd(ADC2, DISABLE);
		ADC_Cmd(ADC3, DISABLE);
}

void USART1_init(uint32_t* addr, uint32_t size){
	
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
	
    GPIO_StructInit(&gpio);
 
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, & gpio);
 
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, & gpio);
 
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
 
		USART_StructInit(&usart);
		usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		usart.USART_BaudRate = USART_BaundRate;	
		USART_Init(USART1, &usart);	
 
		NVIC_EnableIRQ(USART1_IRQn);
		USART_Cmd(USART1, ENABLE);
	
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
}

void USART1_dma_init(uint32_t* addr, uint32_t size){
	
	USART_InitTypeDef USART_InitStructure;
	
	USART_StructInit(&USART_InitStructure);
 	USART_InitStructure.USART_BaudRate = USART_BaundRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
	
	DMA_DeInit(DMA2_Stream7);
	DMA_USART_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
	DMA_USART_InitStructure.DMA_Memory0BaseAddr = (uint32_t)addr;
	DMA_USART_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_USART_InitStructure.DMA_BufferSize = size;
	DMA_USART_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_USART_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_USART_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_USART_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_USART_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_USART_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_USART_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_USART_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_USART_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_USART_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);	
	
	USART_Cmd(USART1, ENABLE);
//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //interrupt for complete recieve data
}
 
void UART1_DMA_run(uint32_t* addr, uint32_t size){
	
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_USART_InitStructure.DMA_Memory0BaseAddr = (uint32_t)addr;
	  DMA_USART_InitStructure.DMA_BufferSize = size;
	  DMA_Init(DMA2_Stream7, &DMA_InitStructure);	
		DMA_Cmd(DMA2_Stream7, ENABLE);
}

void Cycle_UART_tx(uint8_t *d_src, uint32_t size_d){
	uint32_t i;
	
	for(i=0;i<size_d;i++){		
		while(!(USART1->SR & USART_SR_TC));				
		USART1->DR=d_src[i];		
	}
	
}


