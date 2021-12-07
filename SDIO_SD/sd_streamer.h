#ifndef __SDIO_SD_H
#define __SDIO_SD_H

#include "stm32f4xx.h"


#define SD_OPERATION_ERASE          0
#define SD_OPERATION_BLOCK          1
#define SD_OPERATION_MULTI_BLOCK    2 
#define SD_OPERATION_END            3
#define SIZE_DATA_CDG 							30

#define  START_SECTOR_FLASH 				10000

typedef struct
{	

  uint32_t N_block_rec_flash; //= START_SECTOR_FLASH;				// 
  uint32_t N_block_read_flash;			// 
  uint32_t N_block_rec_flash_tmp;
	uint32_t N_block_rec_curr_file;	
  uint32_t Cnt_TIM_Block;
	
  uint8_t fl_start_write_flash; //The flag is showing that the start write flash button was pressing and must be reset after checking! 
  uint32_t cnt_file_write_flash; //The counter is showing how many files is writing or how many times the start write flash button was pressing
	
}control_sd;


typedef struct
{	
	uint32_t cnt_status_write;	
	uint32_t cnt_file_sd;
	uint32_t cnt_status_write_file;
	uint8_t stat[4];
	
	uint16_t data[SIZE_DATA_CDG];
	
}cdg_control_sd;

void SDIO_SD_Init(void);									// Инициаклизация flash
uint8_t SDIO_SD_SingleBlock_Rec(uint8_t*, uint16_t);			// Запись одного блока flash
uint8_t SDIO_SD_TIM_Rec(uint8_t* data);
void SDIO_SD_ReadBlock(uint8_t*, uint16_t);						// Чтение одного блока flash
void SD_Write_Data_Info_Sector0(void);
void SD_Read_Data_Info_Sector0(void);
void cntrl_streamer(uint8_t cntr);

extern cdg_control_sd cntr_sd;
extern control_sd cntrl_sd;

#endif /* __SDIO_SD_H */
