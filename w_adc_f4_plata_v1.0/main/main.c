/*******************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_tim.h"
#include "sd_streamer.h"
#include "adc_module_def.h"
#include "conf_init.h"
#include "stm32f4_discovery_sdio_sd.h"
#include  "sd_stream_cntrl.h"
#include "esp_uart.h"
#include "proc.h"


//================ MAIN ==================================================================================================

int main(){
uint8_t cc = 0;
	
init_proc();
	
//unit_test1();
//unit_test2();
	
wait_sync();
wait_command_init();

while(!cc){
	proc();	
}

//-------------------------------------------------------------------------------	

return 0;

}//end main
 
