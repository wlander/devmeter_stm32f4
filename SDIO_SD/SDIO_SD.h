#ifndef __SDIO_SD_H
#define __SDIO_SD_H

#include "stm32f4xx.h"

void SDIO_SD_Init (void);									// �������������� flash
void SDIO_SD_Erase (void);								// �������� flash
void SDIO_SD_SingleBlock_Rec (void);			// ������ ������ ����� flash
void SDIO_SD_MultiBlock_Rec (void);				// ������ ���������� ������ flash
void SDIO_SD_ReadMultiBlocks (void);			// ������ ���������� ������ flash
void SDIO_SD_ReadBlock (void);						// ������ ������ ����� flash
void SD_Write_Data_Info_Sector0(void);
void SD_Read_Data_Info_Sector0(void);

extern uint32_t N_block_rec_flash;				// ���-�� ���������� ������ � ������ ����
extern uint32_t N_block_read_flash;				// ���-�� ��������� ������ �� ������ ����


#endif /* __SDIO_SD_H */
