#ifndef __SDIO_SD_H
#define __SDIO_SD_H

#include "stm32f4xx.h"

void SDIO_SD_Init (void);									// Инициаклизация flash
void SDIO_SD_Erase (void);								// стирание flash
void SDIO_SD_SingleBlock_Rec (void);			// Запись одного блока flash
void SDIO_SD_MultiBlock_Rec (void);				// Запись нескольких блоков flash
void SDIO_SD_ReadMultiBlocks (void);			// Чтение нескольких блоков flash
void SDIO_SD_ReadBlock (void);						// Чтение одного блока flash
void SD_Write_Data_Info_Sector0(void);
void SD_Read_Data_Info_Sector0(void);

extern uint32_t N_block_rec_flash;				// Кол-во записанных блоков в память флеш
extern uint32_t N_block_read_flash;				// Кол-во считанных блоков из памяти флеш


#endif /* __SDIO_SD_H */
