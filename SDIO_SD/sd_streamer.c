
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "sd_streamer.h"

uint32_t N_block_rec_flash = 0;				// 
uint32_t N_block_read_flash = 0;			// 
uint32_t N_block_rec_flash_tmp = 0;

cdg_cntrl_sd cntr_sd;

//The flag is showing that the start write flash button was pressing and must be reset after checking! 
uint8_t fl_start_write_flash = 0; 
//The counter is showing how many files is writing or how many times the start write flash button was pressing
uint32_t cnt_file_write_flash = 0;

SD_CardInfo Flash_info;		
SD_Error Status = SD_OK;
uint8_t My_Status_SD = 0;
__IO uint32_t SDCardOperation = SD_OPERATION_ERASE;

/* Private functions ---------------------------------------------------------*/

void SDIO_SD_Init(void)
{
  if ((Status = SD_Init()) != SD_OK)	{}
}


void SD_Write_Data_Info_Sector0(void)
{
	uint8_t rxbuff [512];
	uint32_t* ptr_code_file = (uint32_t*)rxbuff;	
	
	//code start file 		
	ptr_code_file[0] = 0x7f555555;	
	ptr_code_file[1] = cnt_file_write_flash;
	ptr_code_file[2] = N_block_rec_flash;	
	
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
		N_block_rec_flash = ptr_code_file[2];
		cnt_file_write_flash = ptr_code_file[1];
	}
	else{
		N_block_rec_flash = 0;
	}
	
	N_block_read_flash = 0;
	
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
							
							if(fl_start_write_flash==1){ //if started new block data (file)
								
									//code start file 		
									ptr_code_file[0] = 0x7f555555;	
									ptr_code_file[1] = cnt_file_write_flash;	
									ptr_code_file[2] = 0;
											
									cnt_file_write_flash++; //inc counter of files
									fl_start_write_flash = 0;
								
							}					
						
						
							Status = SD_WriteBlock(data+i*512, (512*N_block_rec_flash), 512);
							Status = SD_WaitWriteOperation();
							while(SD_GetStatus() != SD_TRANSFER_OK);		
							N_block_rec_flash++;
						
							if(Status != 0){
									My_Status_SD = 2;
									return My_Status_SD;
							}
							else if (N_block_rec_flash == Flash_info.SD_csd.DeviceSize)			// ???? ?????? ?????????, ?? ????????????? ???????
							{
									My_Status_SD = 3;
									return My_Status_SD;
							}
						
					}
					
			}
	}
	
	return 0;
		
}

void cntrl_streamer(uint8_t cntr){
		
		if(cntr==1){
				
		}
		else{
			
			if(N_block_rec_flash_tmp!=N_block_rec_flash){
						 SD_Write_Data_Info_Sector0();
						 N_block_rec_flash_tmp = N_block_rec_flash;				
			}
		
		}

}

