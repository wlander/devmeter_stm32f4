#ifndef __SD_STREAM_CNTRL_H
#define __SD_STREAM_CNTRL_H

#include "stm32f4xx.h"

//#define EN_CNTRL_BUTTON

#define NUM_REFRESH_SD_INFO 200
#define PERIOD_BLINK 20

#define BLINK_CNTRL_PIN GPIO_Pin_4
#define BTN1_CNTRL_PIN GPIO_Pin_6
#define BTN2_CNTRL_PIN GPIO_Pin_7

uint8_t ask_sd_ctr_btn(void);
uint8_t ack_cnt_bttn(void);

extern uint8_t Button_read;
extern uint8_t Button_catch;
extern uint8_t Button_tmp;
extern uint8_t Button_read_cnt;
extern uint8_t fl_button_on;
extern uint8_t Button;
extern uint8_t Reset_Data_Button;
extern uint8_t Reset_Data_Button_cnt;
extern uint8_t Reset_Data_Button_Catch;
extern uint32_t cnt_blink;
extern uint16_t cnt_sector;

#endif /* __SDIO_SD_H */
