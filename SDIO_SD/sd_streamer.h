#ifndef __SDIO_SD_H
#define __SDIO_SD_H

#include "stm32f4xx.h"


#define SD_OPERATION_ERASE          0
#define SD_OPERATION_BLOCK          1
#define SD_OPERATION_MULTI_BLOCK    2 
#define SD_OPERATION_END            3

typedef struct
{	
	uint32_t cnt_status_write;	
	uint32_t cnt_file_sd;
	uint32_t cnt_status_write_all;
	uint32_t stat;
	
	uint32_t data[32];
	
}cdg_cntrl_sd;

void SDIO_SD_Init(void);									// Инициаклизация flash
uint8_t SDIO_SD_SingleBlock_Rec(uint8_t*, uint16_t);			// Запись одного блока flash
void SDIO_SD_ReadBlock(uint8_t*, uint16_t);						// Чтение одного блока flash
void SD_Write_Data_Info_Sector0(void);
void SD_Read_Data_Info_Sector0(void);
void cntrl_streamer(uint8_t cntr);

extern uint32_t N_block_rec_flash;				// Кол-во записанных блоков в память флеш
extern uint32_t N_block_rec_flash_tmp;	
extern uint32_t N_block_read_flash;				// Кол-во считанных блоков из памяти флеш

extern uint8_t fl_start_write_flash;  //The flag is showing that the start write flash button was pressing and must be reset after checking! 
extern uint32_t cnt_file_write_flash; //The counter is showing how many files is writing or how many times the start write flash button was pressing

extern cdg_cntrl_sd cntr_sd;
#endif /* __SDIO_SD_H */
