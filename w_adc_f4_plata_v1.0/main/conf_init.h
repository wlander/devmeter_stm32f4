/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONF_INIT_H
#define __CONF_INIT_H
//----------------------------------------------------------
#include "stm32f4xx.h"
//----------------------------------------------------------
#define USART_BaundRate 230400 //460800
//----------------------------------------------------------

//void pll_config(void);
void RCC_init_all(void);
void GPIO_init_all(void);
void NVIC_Configuration(void);

void ADC_triple_dma_init(uint32_t* ptr_adc_data1, uint32_t* ptr_adc_data2, uint32_t buf_size, uint8_t en_pp);
void Set_ADC_SampleTime(uint8_t num);
void ADC_triple_dma_run(void);
void ADC_triple_dma_stop(void);

void USART1_init(uint32_t* addr, uint32_t size);
void USART1_dma_init(uint32_t* addr, uint32_t size);
void UART1_DMA_run(uint32_t* addr, uint32_t size);
void Cycle_UART_tx(uint8_t *d_src, uint32_t size_d);
void exl_header_def(uint8_t* d_src, uint32_t size);
#endif
