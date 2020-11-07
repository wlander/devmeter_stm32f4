
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "SDIO_SD.h"
#include "fifo.h"
#include "Def_Params.h"
//#include "USART.h"
//#include "Encoder.h"
#include "stm32f4xx_gpio.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_uSDCard
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE            512 /* Block Size in Bytes */

#define NUMBER_OF_BLOCKS      100  /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE    (BLOCK_SIZE * NUMBER_OF_BLOCKS)

#define SD_OPERATION_ERASE          0
#define SD_OPERATION_BLOCK          1
#define SD_OPERATION_MULTI_BLOCK    2 
#define SD_OPERATION_END            3


uint8_t fl_start_write_flash = 0;
uint32_t cnt_file_write_flash = 0;

/* Private macro -------------------------------------------------------------*/
//FIFO( MK_TO_COMP_FIFO_SIZE ) MK_to_Comp_fifo;
//extern FIFO( COMP_TO_MK_FIFO_SIZE ) Comp_to_MK_fifo;

/* Private variables ---------------------------------------------------------*/
uint8_t Buffer_Block_Tx[BLOCK_SIZE], Buffer_Block_Rx[BLOCK_SIZE];
uint8_t Buffer_MultiBlock_Tx[MULTI_BUFFER_SIZE], Buffer_MultiBlock_Rx[MULTI_BUFFER_SIZE];
volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;
SD_Error Status = SD_OK;
__IO uint32_t SDCardOperation = SD_OPERATION_ERASE;

uint32_t N_block_rec_flash = 0;				// Кол-во записанных блоков в память флеш
uint32_t N_block_read_flash = 0;			// Кол-во считанных блоков из памяти флеш

SD_CardInfo Flash_info;							// 

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void SD_EraseTest(void);
void SD_SingleBlockTest(void);
void SD_MultiBlockTest(void);
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
TestStatus eBuffercmp(uint8_t* pBuffer, uint32_t BufferLength);

//The flag is showing that the start write flash button was pressing and must be reset after checking! 
extern uint8_t fl_start_write_flash; 
//The counter is showing how many files is writing or how many times the start write flash button was pressing
extern uint32_t cnt_file_write_flash;

extern uint32_t cur_rtc_tr;
extern uint32_t cur_rtc_dr;

/* Private functions ---------------------------------------------------------*/

void SDIO_SD_Init (void)
{
  NVIC_Configuration();
  if ((Status = SD_Init()) != SD_OK)	{}

}

void SDIO_SD_Erase (void)
{
  while((Status == SD_OK) && (SDCardOperation != SD_OPERATION_END) && (SD_Detect()== SD_PRESENT))
	{
		SD_EraseTest();
	}
}
/*void SDIO_SD_SingleBlock_Rec (void)
{
  while((Status == SD_OK) && (SDCardOperation != SD_OPERATION_END) && (SD_Detect()== SD_PRESENT))
	{
		SD_SingleBlockTest();
	}
}
*/

void SDIO_SD_SingleBlock_Rec (void)
{
	uint8_t rxbuff [512];
	uint32_t* ptr_code_file = (uint32_t*)rxbuff;
	uint8_t NumSectorsWtite = 1;
	
	uint32_t rxbuff_size = 512;			// 512 не изменять !!!
	uint16_t i=0;
	uint16_t j=0;
	char Status_Flash = 0;
		
	if (N_block_rec_flash == 0)
	{
		Status_Flash = SD_GetCardInfo(&Flash_info);
		if (Status_Flash != 0)	// Если какой-то косяк, то останавливаем процесс
		{
			//User Code
		}
	}
	if (N_block_rec_flash == Flash_info.SD_csd.DeviceSize)			// Если флешка заполнена, то останавливаем процесс
	{
			//User Code
	}
	
//  for(; !(FIFO_IS_EMPTY( MK_to_Comp_fifo )); )
  for(j=0; j<NumSectorsWtite; j++)			
	{
		//если данных в fifo больше нет то запрещаем это прерывание
		WIM_task &= ~WIM_MK_TO_COMP;

		// Заполните бефер для отправки
		for (i=0; i<rxbuff_size; i++)
		{
			rxbuff[i] = 0;
		}
		
		if(fl_start_write_flash==1){
			
				//code start file 		
				ptr_code_file[0] = 0x7f555555;	
				ptr_code_file[1] = cnt_file_write_flash;	
				ptr_code_file[2] = 0;
			
//				put_rtc_in_buf(rxbuff+3*4); //For RTC if Need
			
				cnt_file_write_flash++;
				fl_start_write_flash = 0;
			
		}
		
		//передаем байт
		if (Status == SD_OK) 
		{
			// Писать блоками по 512 байт по адресу 0
			Status = SD_WriteBlock(rxbuff, (rxbuff_size*N_block_rec_flash), (uint16_t)rxbuff_size);
			// Проверьте, закончена ли передача
			Status = SD_WaitWriteOperation();
			while(SD_GetStatus() != SD_TRANSFER_OK);
			N_block_rec_flash++;
		}
  }

	//раз в fifo данных меньше чем 512, то запрещаем это прерывание
	WIM_task &= ~WIM_MK_TO_COMP;
}

void SD_Write_Data_Info_Sector0(void)
{
	uint8_t rxbuff [512];
	uint32_t* ptr_code_file = (uint32_t*)rxbuff;	
	
	//code start file 		
	ptr_code_file[0] = 0x7f555555;	
	ptr_code_file[1] = cnt_file_write_flash;
	ptr_code_file[2] = N_block_rec_flash;
	
//	put_rtc_in_buf(rxbuff+3*4);		for RTC if Need
	
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

void SDIO_SD_MultiBlock_Rec (void)
{
  while((Status == SD_OK) && (SDCardOperation != SD_OPERATION_END) && (SD_Detect()== SD_PRESENT))
	{
		SD_MultiBlockTest();
	}
}
	
void SDIO_SD_ReadMultiBlocks_ (void)
{
 if (Status == SD_OK) {
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);

    // Check if the Transfer is finished 
    Status = SD_WaitReadOperation();

    // Wait until end of DMA transfer 
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }
}
	
void SDIO_SD_ReadMultiBlocks (void)
{
//SD_Error SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
	uint8_t rxbuff [512];
	uint32_t rxbuff_size = 512;			// 512 не изменять !!!
	uint16_t i=0;
	uint16_t tt=0;

	// Дождитесь окончания передачи DMA
	while(SD_GetStatus() != SD_TRANSFER_OK);

	if (N_block_read_flash < N_block_rec_flash)
	{
		if (Status == SD_OK) {
			Status = SD_ReadMultiBlocks(rxbuff, (N_block_read_flash*rxbuff_size), rxbuff_size, 1);
//			for(i=0; i<rxbuff_size; i++)			{	if ( !FIFO_IS_FULL( MK_to_Comp_fifo ) )			FIFO_PUSH( MK_to_Comp_fifo, rxbuff[i] );	}		// Отправляем данные в FIFO

			// Проверьте закончена ли передача
			Status = SD_WaitReadOperation();
			
			//!!! Write Sector to the User Bufer!!!
			for(i=0; i<rxbuff_size; i++)			
			{	
//				if ( !FIFO_IS_FULL( MK_to_Comp_fifo ) )			
//				{
//					FIFO_PUSH( MK_to_Comp_fifo, rxbuff[i] );	
//				}
			}		// Отправляем данные в FIFO


			// Дождитесь окончания передачи DMA
//			while(SD_GetStatus() != SD_TRANSFER_OK);
			N_block_read_flash++;
			WIM_task |= WIM_MK_TO_COMP;		// Устанавливаем флаг на выдачу данных
		}
	}
	else		// if (FIFO_IS_EMPTY(MK_to_Comp_fifo) == 1)
	{
	  WIM_task &= ~WIM_FLASH_TO_COMP;			// Сбрасываем флаг	
		N_block_rec_flash = 0;
		N_block_read_flash = 0;
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	}
}
	
void SDIO_SD_ReadBlock (void)
{
  if (Status == SD_OK) {
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(Buffer_Block_Rx, 0x00, BLOCK_SIZE);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int SDIO_SD (void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

  /* Initialize LEDs available on STM324xG-EVAL board *************************/
//  STM_EVAL_LEDInit(LED3);
//  STM_EVAL_LEDInit(LED4);
//  STM_EVAL_LEDInit(LED5);
//  STM_EVAL_LEDInit(LED6);  

  /* Interrupt Config */
  NVIC_Configuration();

  /*------------------------------ SD Init ---------------------------------- */
  if ((Status = SD_Init()) != SD_OK)
	{
//    STM_EVAL_LEDOn(LED6); 
  }

  while((Status == SD_OK) && (SDCardOperation != SD_OPERATION_END) && (SD_Detect()== SD_PRESENT))
  {
    switch(SDCardOperation)
    {
      /*-------------------------- SD Erase Test ---------------------------- */
      case (SD_OPERATION_ERASE):
      {
        SD_EraseTest();
        SDCardOperation = SD_OPERATION_BLOCK;
        break;
      }
      /*-------------------------- SD Single Block Test --------------------- */
      case (SD_OPERATION_BLOCK):
      {
        SD_SingleBlockTest();
        SDCardOperation = SD_OPERATION_MULTI_BLOCK;
        break;
      }       
      /*-------------------------- SD Multi Blocks Test --------------------- */
      case (SD_OPERATION_MULTI_BLOCK):
      {
        SD_MultiBlockTest();
        SDCardOperation = SD_OPERATION_END;
        break;
      }              
    }
  }
  
      /*-------------------------- Чтение SD карты --------------------- */
 if (Status == SD_OK) {
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);

    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();

    /* Wait until end of DMA transfer */
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }


  /* Infinite loop */
  while (1);
}


void SD_EraseTest(void)
{
  /*------------------- Block Erase ------------------------------------------*/
  if (Status == SD_OK) {
    /* Erase NumberOfBlocks Blocks of WRITE_BL_LEN(512 Bytes) */
    Status = SD_Erase(0x00, (BLOCK_SIZE * NUMBER_OF_BLOCKS));
  }

  if (Status == SD_OK) {
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);

    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();

    /* Wait until end of DMA transfer */
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of erased blocks */
  if (Status == SD_OK) {
    EraseStatus = eBuffercmp(Buffer_MultiBlock_Rx, MULTI_BUFFER_SIZE);
  }
  
  if (EraseStatus == PASSED) {
    STM_EVAL_LEDOn(LED3);
  } else {
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOn(LED6);    
  }
}

/**
  * @brief  Tests the SD card Single Blocks operations.
  * @param  None
  * @retval None
  */
void SD_SingleBlockTest(void)
{
  /*------------------- Block Read/Write --------------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_Block_Tx, BLOCK_SIZE, 0x320F);

  if (Status == SD_OK) {
    /* Write block of 512 bytes on address 0 */
    Status = SD_WriteBlock(Buffer_Block_Tx, 0x00, BLOCK_SIZE);
    /* Check if the Transfer is finished */
    Status = SD_WaitWriteOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  if (Status == SD_OK) {
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(Buffer_Block_Rx, 0x00, BLOCK_SIZE);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of written data */
  if (Status == SD_OK) {
    TransferStatus1 = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BLOCK_SIZE);
  }
  
  if (TransferStatus1 == PASSED) {
    STM_EVAL_LEDOn(LED4);
  } else {
    STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOn(LED6);    
  }
}

/**
  * @brief  Tests the SD card Multiple Blocks operations.
  * @param  None
  * @retval None
  */
void SD_MultiBlockTest(void)
{
  /*--------------- Multiple Block Read/Write ---------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_MultiBlock_Tx, MULTI_BUFFER_SIZE, 0x0);

  if (Status == SD_OK) {
    /* Write multiple block of many bytes on address 0 */
    Status = SD_WriteMultiBlocks(Buffer_MultiBlock_Tx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);
    /* Check if the Transfer is finished */
    Status = SD_WaitWriteOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  if (Status == SD_OK) {
    /* Read block of many bytes from address 0 */
    Status = SD_ReadMultiBlocks(Buffer_MultiBlock_Rx, 0x00, BLOCK_SIZE, NUMBER_OF_BLOCKS);
    /* Check if the Transfer is finished */
    Status = SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
  }

  /* Check the correctness of written data */
  if (Status == SD_OK) {
    TransferStatus2 = Buffercmp(Buffer_MultiBlock_Tx, Buffer_MultiBlock_Rx, MULTI_BUFFER_SIZE);
  }
  
  if(TransferStatus2 == PASSED) {
    STM_EVAL_LEDOn(LED5);
  } else {
    STM_EVAL_LEDOff(LED5);
    STM_EVAL_LEDOn(LED6);    
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2) {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the Buffer to fill
  * @param  BufferLength: size of the buffer to fill
  * @param  Offset: first value to fill on the Buffer
  * @retval None
  */
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLength; index++) {
    pBuffer[index] = index + Offset;
  }
}

/**
  * @brief  Checks if a buffer has all its values are equal to zero.
  * @param  pBuffer: buffer to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer values are zero
  *         FAILED: At least one value from pBuffer buffer is different from zero.
  */
TestStatus eBuffercmp(uint8_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    /* In some SD Cards the erased state is 0xFF, in others it's 0x00 */
    if ((*pBuffer != 0xFF) && (*pBuffer != 0x00)) {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}





