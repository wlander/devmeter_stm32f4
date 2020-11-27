#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "esp_uart.h"
#include "esp_lib.h"
#include "fifo_o.h"

//AT strings for request to ESP
static char AT_Connect_UDP[] = {"AT+CIPSTART=\"UDP\",\"192.168.4.2\",8888,1112,2\r\n"};
static char AT_Req_OK[] = {"AT\r\n"};
static char AT_RST[] = {"AT+RST\r\n"};
static char AT_CIPSEND_S[] = {"AT+CIPSEND=80\r\n"};
static char AT_SET_ATE0[] = {"ATE0\r\n"};

//AT strings of responses from ESP
static char AT_Resp_OK[] = {"OK"};
static char AT_Resp_ERROR[] = {"ERROR"};
static char AT_Resp_ARD_CNCT[] = {"ALREADY CONNECT"};
static char AT_Resp_Inv_TR[] = {">"};
static char AT_Resp_rcv_data[] = {"+IPD"};

char recv_msg_buf[MAX_RCV_MSG];
uint8_t test_data[80];

FIFO_T esp_uart_fifo_msg;
uint8_t Fifo_overflow = 0;

uint32_t cnt_rcv_esp_uart = 0;
uint32_t cnt_rcv_esp_uart_msg = 0;
uint32_t cnt_obr_esp_uart_msg = 0;

void ESP_UART_IRQ_Handler(void){
	uint8_t tmp_b;
	tmp_b = ESP_UART->DR;
	
	if ( !FIFO_IS_FULL( esp_uart_fifo_msg )){
			FIFO_PUSH( esp_uart_fifo_msg, tmp_b );
			if(tmp_b==FINAL_RCV_MSG_CHAR) cnt_rcv_esp_uart_msg++;
	}
	else{
			Fifo_overflow = 1;
	}
	
	cnt_rcv_esp_uart++;	
	USART_ClearITPendingBit(ESP_UART, USART_IT_RXNE);
}


//init uart (should select uart params in the esp_uart.h)
uint8_t init_esp_uart(void)     
{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;	
	
//------------ UART clock enable ------------------------------------------
	 RCC_APB1PeriphClockCmd(ESP_UART_CLK, ENABLE);
	 RCC_APB2PeriphClockCmd(ESP_UART_CLK, ENABLE);
//--------------------------------------------------------------------------

    GPIO_StructInit(&gpio);
 
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = ESP_UART_TX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ESP_UART_TX_GPIO_PORT, & gpio);
 
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = ESP_UART_RX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ESP_UART_RX_GPIO_PORT, & gpio);
 
		GPIO_PinAFConfig(ESP_UART_TX_GPIO_PORT, ESP_UART_TX_SOURCE, ESP_UART_GPIO_AF);
    GPIO_PinAFConfig(ESP_UART_RX_GPIO_PORT, ESP_UART_RX_SOURCE , ESP_UART_GPIO_AF);
 
		USART_StructInit(&usart);
		usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		usart.USART_BaudRate = ESP_USART_BaundRate;	
		USART_Init(ESP_UART, &usart);	
 
		NVIC_EnableIRQ(ESP_UART_IRQn);
		USART_Cmd(ESP_UART, ENABLE);
	
		USART_ITConfig(ESP_UART, USART_IT_RXNE, ENABLE);
	
	 return 0;
}

uint8_t get_uart_msg(char* out_d){
	uint16_t i = 0;
	uint16_t k = 0;
	uint8_t d = 0;
	uint8_t st = 0;
	char mbuf[MAX_RCV_MSG];
	
	if(cnt_rcv_esp_uart_msg!=0){

		d = FIFO_FRONT( esp_uart_fifo_msg );
		FIFO_POP( esp_uart_fifo_msg );
		mbuf[k++] = d;
		
		while(d!=FINAL_RCV_MSG_CHAR)
		{
			d = FIFO_FRONT( esp_uart_fifo_msg );
			FIFO_POP( esp_uart_fifo_msg );
			mbuf[k++] = d;
		}
		
		st = parse_esp_uart_msg(mbuf);
		
		cnt_rcv_esp_uart_msg--;
		cnt_obr_esp_uart_msg++;
		
		if(st!=0){
			for(i=0;i<k;i++) out_d[i] = mbuf[i];
			return k;
		}
		else{
			return 0;
		}
		
		
	}
	else{
		return 0;
	}
}

void clear_fifo_esp(void){
	FIFO_FLUSH(esp_uart_fifo_msg);
}

void esp_uart_tr(uint8_t *d_src, uint32_t size_d){
	uint32_t i;
	
	for(i=0;i<size_d;i++){		
		while(!(ESP_UART->SR & USART_SR_TC));				
		ESP_UART->DR=d_src[i];		
	}

	
}

uint8_t parse_esp_uart_msg(char* msg){
	uint16_t i;
	uint16_t check_sum = 0;
	
	//check AT response = "OK"
	for(i=0;i<sizeof(AT_Resp_OK)-1;i++) check_sum+=msg[i]-AT_Resp_OK[i];
	if(check_sum==0) return ESP_RESP_OK;
	
	check_sum = 0;
	//check AT response = "ERROR"
	for(i=0;i<sizeof(AT_Resp_ERROR)-1;i++) check_sum+=msg[i]-AT_Resp_ERROR[i];
	if(check_sum==0) return ESP_RESP_ERROR;
	
	check_sum = 0;
	//check AT response = ">"
	check_sum = msg[i]-AT_Resp_Inv_TR[i];
	if(check_sum==0) return ESP_RESP_INV_TR	;	

	check_sum = 0;
	//check AT response = "+IPD"
	for(i=0;i<sizeof(AT_Resp_rcv_data)-1;i++) check_sum+=msg[i]-AT_Resp_rcv_data[i];
	if(check_sum==0) return ESP_RESP_RCV_D;
	
	check_sum = 0;
	//check AT response = "ALREADY CONNECT"
	for(i=0;i<sizeof(AT_Resp_ARD_CNCT)-1;i++) check_sum+=msg[i]-AT_Resp_ARD_CNCT[i];
	if(check_sum==0) return ESP_ALRDY_CNNCT;	
	
	return 0;
	
}

uint8_t esp_rst(void){
	esp_uart_tr((uint8_t*)AT_RST, sizeof(AT_RST));
	return 0;
}

uint8_t esp_set_ate0(void){
	esp_uart_tr((uint8_t*)AT_SET_ATE0, sizeof(AT_SET_ATE0));
	return 0;
}

uint8_t esp_udp_connect(void){
	esp_uart_tr((uint8_t*)AT_Connect_UDP, sizeof(AT_Connect_UDP));
	return 0;
}

uint8_t esp_ask_ok(void){
	esp_uart_tr((uint8_t*)AT_Req_OK, sizeof(AT_Req_OK)-1);
	return 0;
}

uint8_t esp_start_uart_tr(void){
	esp_uart_tr((uint8_t*)AT_CIPSEND_S, sizeof(AT_CIPSEND_S));
	return 0;
}

uint8_t esp_unit_test1(){
	uint32_t i;
	uint8_t type_resp = 0;
	
	init_esp_uart();
	for(i=0;i<10000000;i++){i=i;}
	
	//Asking ESP is "OK"?
	esp_ask_ok();
	
	while(!get_uart_msg(recv_msg_buf));
	
	type_resp = parse_esp_uart_msg(recv_msg_buf);
	
	if(type_resp==ESP_RESP_ERROR) return 0; //if pesponse "ERROR"

	//Set ATE0	
	esp_set_ate0();
	
	while(!get_uart_msg(recv_msg_buf));

	//Asking ESP is "OK"?
	esp_ask_ok();
	
	while(!get_uart_msg(recv_msg_buf));

	//Connecting UDP
	esp_udp_connect();
	for(i=0;i<100000000;i++){i=i;}
	
	while(!get_uart_msg(recv_msg_buf));
	
	type_resp = parse_esp_uart_msg(recv_msg_buf);
	
	if(type_resp==ESP_RESP_ERROR) return 0; //if pesponse "ERROR"
	
	//AT for starting message 
	esp_start_uart_tr();
	
	while(!get_uart_msg(recv_msg_buf));
	
	type_resp = parse_esp_uart_msg(recv_msg_buf);
	
	if(type_resp==ESP_RESP_ERROR) return 0; //if pesponse "ERROR"

	else if(type_resp==ESP_RESP_INV_TR){	
		//send data of message
		esp_uart_tr(test_data, 80);
		while(!get_uart_msg(recv_msg_buf));
	}
	
	return 0xFF;
	
}
