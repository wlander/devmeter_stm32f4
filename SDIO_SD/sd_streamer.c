/********************************************************************************
			
  * @file    proc.c
  * @author  Evgeny Suskov
  * @version V1.1.1
  * @date    10-10-2020
  * @brief   This file implements the following functions: 
  *           + Initialization SD card (SDIO)
  *           + Read and write data to sectors SD card
  *     			+ separation of data into SD card in files (blocks) using conditional operator (0x7f555555)
	*						
  *  COPYRIGHT (C) 2011 - 2021, Evgeny Suskov, Dmitry Kudryashov.
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License GPLv3+ as published by
  *  the Free Software Foundation.
  * software distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied
*****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "sd_streamer.h"

cdg_control_sd cntr_sd;
control_sd cntrl_sd;

SD_CardInfo Flash_info;		
SD_Error Status = SD_OK;
uint8_t My_Status_SD = 0;
__IO uint32_t SDCardOperation = SD_OPERATION_ERASE;

/* Private functions ---------------------------------------------------------*/

void SDIO_SD_Init(void)
{
  if ((Status = SD_Init()) != SD_OK)	{}
		
	cntrl_sd.N_block_rec_flash = START_SECTOR_FLASH;

}


void SD_Write_Data_Info_Sector0(void)
{
	uint8_t rxbuff [512];
	uint32_t* ptr_code_file = (uint32_t*)rxbuff;	
	
	//code start file 		
	ptr_code_file[0] = 0x7f555555;	
	ptr_code_file[1] = cntrl_sd.cnt_file_write_flash;
	ptr_code_file[2] = cntrl_sd.N_block_rec_flash;	
	ptr_code_file[3] = cntrl_sd.Cnt_TIM_Block;
	
	if(Status == SD_OK) 
	{

		Status = SD_WriteBlock(rxbuff, 0, 512);
		Status = SD_WaitWriteOperation();
		while(SD_GetStatus() != SD_TRANSFER_OK);

	}
	
}

void SD_Read_Data_Info_Sector0(void){
	
	uint8_t rxbuff [512];
	uint32_t* ptr_code_file = (uint32_t*)rxbuff;	
	
  if (Status == SD_OK) {
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(rxbuff, 0x00, 512);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }
	
	if(ptr_code_file[0]==0x7f555555){
		cntrl_sd.N_block_rec_flash = ptr_code_file[2];
		cntrl_sd.cnt_file_write_flash = ptr_code_file[1];
		cntrl_sd.Cnt_TIM_Block = ptr_code_file[3];
	}
	else{
		cntrl_sd.N_block_rec_flash = START_SECTOR_FLASH;
		cntrl_sd.Cnt_TIM_Block = 0;
	}
	
	cntrl_sd.N_block_read_flash = START_SECTOR_FLASH;
	
}


void SDIO_SD_ReadBlock(uint8_t* data, uint16_t num_s)
{
  if (Status == SD_OK) {
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(data, 512*num_s, 512);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }
}


uint8_t SDIO_SD_SingleBlock_Rec(uint8_t* data, uint16_t num_s)
{
uint16_t i;
char Status_Flash = 0;
uint32_t* ptr_code_file = (uint32_t*)data;
	
	if(My_Status_SD==0){
		
			for(i=0;i<num_s;i++){
						
					Status_Flash = SD_GetCardInfo(&Flash_info);
					if (Status_Flash != 0)	// 
					{
							My_Status_SD = 1;
							return My_Status_SD;
					}	
					else{ 
							
							if(cntrl_sd.fl_start_write_flash==1){ //if started new block data (file)
								
									//code start file 		
									ptr_code_file[0] = 0x7f555555;	
									ptr_code_file[1] = cntrl_sd.cnt_file_write_flash;	
									ptr_code_file[2] = 0;
											
									cntrl_sd.cnt_file_write_flash++; //inc counter of files
									cntrl_sd.fl_start_write_flash = 0;
								
							}					
						
						
							Status = SD_WriteBlock(data+i*512, (512*cntrl_sd.N_block_rec_flash), 512);
							Status = SD_WaitWriteOperation();
							while(SD_GetStatus() != SD_TRANSFER_OK);		
							cntrl_sd.N_block_rec_flash++;
						  cntrl_sd.N_block_rec_curr_file++;
							if(Status != 0){
									My_Status_SD = 2;
									return My_Status_SD;
							}
							else if (cntrl_sd.N_block_rec_flash == Flash_info.SD_csd.DeviceSize)			// ???? ?????? ?????????, ?? ????????????? ???????
							{
									My_Status_SD = 3;
									return My_Status_SD;
							}
						
					}
					
			}
	}
	
	return 0;
		
}


uint8_t SDIO_SD_TIM_Rec(uint8_t* data)
{
char Status_Flash = 0;
	
	if(My_Status_SD==0){
		
					Status_Flash = SD_GetCardInfo(&Flash_info);
					if (Status_Flash != 0)	// 
					{
							My_Status_SD = 1;
							return My_Status_SD;
					}	
					else{ 
																							
							Status = SD_WriteBlock(data, (512*cntrl_sd.Cnt_TIM_Block), 512);
							Status = SD_WaitWriteOperation();
							while(SD_GetStatus() != SD_TRANSFER_OK);		
							cntrl_sd.Cnt_TIM_Block++;
						
							if(Status != 0){
									My_Status_SD = 2;
									return My_Status_SD;
							}
							else if (cntrl_sd.Cnt_TIM_Block == Flash_info.SD_csd.DeviceSize)			// ???? ?????? ?????????, ?? ????????????? ???????
							{
									My_Status_SD = 3;
									return My_Status_SD;
							}
						
					}
					
	}
	
	return 0;
		
}

void cntrl_streamer(uint8_t cntr){
		
		if(cntr==1){
				
		}
		else{
			
			if(cntrl_sd.N_block_rec_flash_tmp!=cntrl_sd.N_block_rec_flash){
				 SD_Write_Data_Info_Sector0();
				 cntrl_sd.N_block_rec_flash_tmp = cntrl_sd.N_block_rec_flash;				
			}
		
		}

}

