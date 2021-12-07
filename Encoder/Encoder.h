#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

void Encoder_Init (void);
//void Encoder_DeInit (void);
void Encoder_on (void);
void Encoder_off (void);

void Processing_TIM1_chan3(int64_t counter_CNT_chan_3, int64_t counter_CNT_chan_3_from_overflow);
void Processing_TIM1_chan1(int64_t counter_CNT_chan_1, int64_t counter_CNT_chan_1_from_overflow);

void Init_All_Timers (void);			// Инициализация и включение всех таймеров
void DeInit_All_Timers (void);			// Выключение всех таймеров

void Led_Init (void);
void Button_Init (void);

#define PIN_ENCODER1 (Pe)

#define ENCODER_INPUT1	GPIO_Pin_9
#define ENCODER_INPUT1_PORT	GPIOE

#define ENCODER_INPUT2	GPIO_Pin_13
#define ENCODER_INPUT2_PORT	GPIOE


//Status flags
#define WIM_MK_TO_COMP						(1 << 0)	// Buffer isn't empty
#define WIM_INPUT_SETTING					(1 << 1)	// Codogramm of custom is recieved
#define WIM_FLASH_TO_COMP					(1 << 2)	// Flash to PC mode
#define WIM_ENCODER_OFF						(1 << 3)	// Encoder on/off
#define WIM_MODE_USB							(1 << 4)	// it at that
#define WIM_MODE_UART							(1 << 5)	// 
#define WIM_MODE_FLASH_WRITE			(1 << 6)	// 
#define WIM_MODE_FLASH_ERASE			(1 << 7)	// ????? ???????? ????
#define WIM_FIFO_FILLED						(1 << 8)	// FIFO is filled
			

#endif /* __ENCODER_H */
