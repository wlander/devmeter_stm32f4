#ifndef __ESP_UART_H
#define __ESP_UART_H

#include "stm32f4xx.h"
#include "sd_streamer.h"

#define ESP_UART                        USART3
#define ESP_UART_CLK                    RCC_APB1Periph_USART3
#define ESP_UART_TX_PIN                 GPIO_Pin_10
#define ESP_UART_TX_GPIO_PORT           GPIOC
#define ESP_UART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define ESP_UART_TX_SOURCE              GPIO_PinSource10
#define ESP_UART_GPIO_AF                GPIO_AF_USART3
#define ESP_UART_RX_PIN                 GPIO_Pin_11
#define ESP_UART_RX_GPIO_PORT           GPIOC
#define ESP_UART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define ESP_UART_RX_SOURCE              GPIO_PinSource11
#define ESP_UART_IRQn                   USART3_IRQn 
#define ESP_UART_IRQ_Handler            USART3_IRQHandler

#define FINAL_RCV_MSG_CHAR							0x0A
#define MAX_RCV_MSG											128
#define TR_MSG_SIZE											(sizeof(cdg_cntrl_sd))
#define ESP_TR_FIFO_ISFULL							1
#define ESP_TR_FIFO_ISFREE							0
#define ESP_UDP_INIT_ERROR							1

#define ESP_USART_BaundRate							115200

#define NUM_DELAY_TR			16

#define ESP_RESP_OK				1
#define ESP_RESP_ERROR		2
#define ESP_RESP_INV_TR		3
#define ESP_RESP_RCV_D		4
#define ESP_ALRDY_CNNCT		5
#define EXT_CONF_BYTE			7
#define ESP_SEND_OK				8

uint8_t init_esp_uart(void);
uint8_t get_uart_msg(char*);
void clear_fifo_esp(void);
void esp_uart_tr(uint8_t*, uint32_t);
uint8_t parse_esp_uart_msg(char*);

uint8_t esp_udp_connect(void);
uint8_t esp_udp_disconnect(void);
uint8_t esp_ask_ok(void);
uint8_t esp_start_uart_tr(void);
uint8_t esp_start_uart_tr2(void);
uint8_t esp_rst(void);
uint8_t esp_set_ate0(void);
uint8_t esp_init_udp(void);
void clear_fifo_esp(void);
void clear_fifo_esp_tr(void);
uint8_t esp_uart_msg_tr_start(uint8_t *d_src, uint32_t size_d);
uint8_t esp_uart_msg_tr_continue(void);
uint8_t esp_msg_tr_status(void);

uint8_t put_esp_uart_fifo_tr(uint8_t*, uint32_t);
uint8_t esp_uart_msg_tr(void);
uint8_t esp_check_rcv_msg(void);
//unit test
uint8_t esp_unit_test1(void);

//#define UNIT_TEST1
//#define UNIT_TEST2

#endif
