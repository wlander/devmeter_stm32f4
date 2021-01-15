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
static char AT_CIPSEND_S[] = {"AT+CIPSEND=216\r\n"};
static char AT_CIPSEND_S2[] = {"AT+CIPSEND=16\r\n"};
static char AT_SET_ATE0[] = {"ATE0\r\n"};
static char AT_Close_UDP[] = {"AT+CIPCLOSE\r\n"};

//AT strings of responses from ESP
static char AT_Resp_OK[] = {"OK"};
static char AT_Resp_ERROR[] = {"ERROR"};
static char AT_Resp_ARD_CNCT[] = {"ALREADYCONNECT"};
static char AT_Resp_Inv_TR[] = {">"};
static char AT_Resp_rcv_data[] = {"+IPD"};
static char AT_Resp_send_ok[] = {"SENDOK"};

char recv_msg_buf[MAX_RCV_MSG];
uint8_t tr_msg_buf[TR_MSG_SIZE];
uint8_t test_data[80];

FIFO_T esp_uart_fifo_msg;
FIFO_T esp_uart_fifo_tr;
uint8_t Fifo_overflow = 0;

uint32_t cnt_rcv_esp_uart = 0;
volatile uint32_t cnt_rcv_esp_uart_msg = 0;
uint32_t cnt_obr_esp_uart_msg = 0;
uint32_t cnt_send_ok = 0;
uint32_t size_msg_tr = 0;

volatile uint8_t fl_rcv_data = 0;
uint8_t task_tr = 0;
uint8_t tst_flg = 0;
uint8_t cnt_tstt = 0;

void ESP_UART_IRQ_Handler(void){
	uint8_t tmp_b;
	tmp_b = ESP_UART->DR;


	if(tmp_b==RECV_DATA){
		fl_rcv_data = 1;
	}	
	else if ((FIFO_COUNT(esp_uart_fifo_msg)<FIFO_SIZE(esp_uart_fifo_msg)) && (tmp_b!=0x20) && (tmp_b!=0x0D)){

			FIFO_PUSH( esp_uart_fifo_msg, tmp_b );
		
			if(tmp_b==FINAL_RCV_MSG_CHAR){
				cnt_rcv_esp_uart_msg++;
			}
			else if(tmp_b==0x3E){
				FIFO_PUSH(esp_uart_fifo_msg, FINAL_RCV_MSG_CHAR);
				cnt_rcv_esp_uart_msg++;
			}
			
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

uint8_t esp_init_udp(void){
	uint32_t i;
	uint8_t type_resp = 0;
	
	//Asking ESP is "OK"?
	esp_ask_ok();
	
	while(!get_uart_msg(recv_msg_buf));
	
	type_resp = parse_esp_uart_msg(recv_msg_buf);
	
	if(type_resp==ESP_RESP_ERROR) return 1; //if pesponse "ERROR"

	//Set ATE0	
	esp_set_ate0();
	
	while(!get_uart_msg(recv_msg_buf));

	//Asking ESP is "OK"?
	esp_ask_ok();
	
	while(!get_uart_msg(recv_msg_buf));

	//Connecting UDP
	esp_udp_connect();
	for(i=0;i<10000000;i++){i=i;}
	
	while(!get_uart_msg(recv_msg_buf));
	
	type_resp = parse_esp_uart_msg(recv_msg_buf);
	
	if(type_resp==ESP_RESP_ERROR) return 1; //if pesponse "ERROR"
	
	return 0;
}

uint8_t get_fl_rcv_data_stat(void){
	return fl_rcv_data;
}

void reset_fl_rcv_data_stat(void){
	fl_rcv_data = 0;
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
void clear_fifo_esp_tr(void){
	FIFO_FLUSH(esp_uart_fifo_tr);
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
	check_sum = msg[0]-AT_Resp_Inv_TR[0];
	if(check_sum==0){
		task_tr = 2;
		return ESP_RESP_INV_TR;	
	}
	check_sum = 0;
	//check AT response = "+IPD"
	for(i=0;i<sizeof(AT_Resp_rcv_data)-1;i++) check_sum+=msg[i]-AT_Resp_rcv_data[i];
	if(check_sum==0){
		if(msg[5]==0x31) return EXT_CONF_BYTE;
		else return 0;//ESP_RESP_RCV_D; !!! Vremenno otkl nuzno sdelat kopirovanie prymo otsyuda v drugoi bufer
	}
	check_sum = 0;
	//check AT response = "ALREADY CONNECT"
	for(i=0;i<sizeof(AT_Resp_ARD_CNCT)-1;i++) check_sum+=msg[i]-AT_Resp_ARD_CNCT[i];
	if(check_sum==0) return ESP_ALRDY_CNNCT;	
	
	check_sum = 0;
	//check AT response = "SEND OK"
	for(i=0;i<sizeof(AT_Resp_send_ok)-1;i++) check_sum+=msg[i]-AT_Resp_send_ok[i];
	if(check_sum==0){
		task_tr = 3;
		cnt_send_ok++;
		return ESP_SEND_OK;	
	}
	
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

uint8_t esp_udp_disconnect(void){
	esp_uart_tr((uint8_t*)AT_Close_UDP, sizeof(AT_Close_UDP));
	return 0;
}

uint8_t esp_ask_ok(void){
	esp_uart_tr((uint8_t*)AT_Req_OK, sizeof(AT_Req_OK)-1);
	return 0;
}

uint8_t put_esp_uart_fifo_tr(uint8_t *d_src, uint32_t size_d){
	uint32_t i;
	
	if(FIFO_COUNT(esp_uart_fifo_tr)<FIFO_SIZE(esp_uart_fifo_tr)){
		for(i=0;i<size_d;i++){
			FIFO_PUSH( esp_uart_fifo_tr, d_src[i] );
		}
		return 0;
	}
	else{
		return 1;
	}	
	
}

uint8_t esp_uart_msg_tr(void){
	uint32_t i;
	
	if(!FIFO_IS_EMPTY(esp_uart_fifo_tr)){
		
		if(task_tr==0){
			task_tr = 1;
			esp_start_uart_tr();
		}
		else if(task_tr==2){
					
			for(i=0;i<TR_MSG_SIZE;i++){
				tr_msg_buf[i] = FIFO_FRONT( esp_uart_fifo_tr );
				FIFO_POP( esp_uart_fifo_tr );
			}

			esp_uart_tr(tr_msg_buf, TR_MSG_SIZE);
			task_tr=0;				
		
		}		
		
		return 1;
	}
	else{
		return 0;
	}
}

uint8_t esp_uart_msg_tr_start(uint8_t *d_src, uint32_t size_d){
	uint32_t i;
		
	if(task_tr==0){
		task_tr = 1;
		size_msg_tr = size_d;		
		for(i=0;i<size_d;i++) tr_msg_buf[i] = d_src[i];
		if(size_d==16) esp_start_uart_tr2();
		else esp_start_uart_tr();
	}
		
	return task_tr;
}

uint8_t esp_uart_msg_tr_continue(void){
		
	if(task_tr==2){					
		esp_uart_tr(tr_msg_buf, size_msg_tr);				
	}	
	else if(task_tr==3){
		task_tr=0;
	}
		
	return task_tr;
}

uint8_t esp_msg_tr_status(void){
	return task_tr;
}

uint8_t esp_start_uart_tr(void){
	esp_uart_tr((uint8_t*)AT_CIPSEND_S, sizeof(AT_CIPSEND_S));
	return 0;
}

uint8_t esp_start_uart_tr2(void){
	esp_uart_tr((uint8_t*)AT_CIPSEND_S2, sizeof(AT_CIPSEND_S2));
	return 0;
}

uint8_t esp_check_rcv_msg(void){
	uint32_t len_msg;
	uint8_t type_resp = 0;
	len_msg = get_uart_msg(recv_msg_buf);
	if(len_msg!=0){
		
		type_resp = parse_esp_uart_msg(recv_msg_buf); //check response of AT command
		
		if(type_resp==EXT_CONF_BYTE){ //if msg is not AT command than it is conf byte from control device (PC)
			return recv_msg_buf[6]; //return conf byte
		}
		
	}
	return 0;
}

uint8_t esp_unit_test1(void){
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
	

while(1){	
	
	tst_flg = 0;
	while(!tst_flg);
	
	//AT for starting message 
	esp_start_uart_tr();
	
	type_resp = 0;
	
	while(type_resp!=ESP_RESP_INV_TR){		
		while(!get_uart_msg(recv_msg_buf));
		type_resp = parse_esp_uart_msg(recv_msg_buf);
	}
	
	test_data[0] = cnt_tstt;
	test_data[1] = 2;
	test_data[2] = 3;
	test_data[3] = 4;
	
	if(type_resp==ESP_RESP_ERROR) return 0; //if pesponse "ERROR"
	
	else if(type_resp==ESP_RESP_INV_TR){	
		//send data of message
		for(i=0;i<1000000;i++){i=i;}
		esp_uart_tr(test_data, 4);
		while(!get_uart_msg(recv_msg_buf));
	}
	
	for(i=0;i<10000;i++){i=i;}
	
	if(tst_flg==2) break;
	
}

	esp_udp_disconnect();
	while(!get_uart_msg(recv_msg_buf));
	
	return 0xFF;
	
}
