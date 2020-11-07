#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4_discovery.h"

/*#if defined (USE_STM324xG_EVAL)
  #include "stm324xg_eval.h"

#elif defined (USE_STM324x7I_EVAL) 
  #include "stm324x7i_eval.h"

#else
 #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif
*/
// ��������� ���������
#define WIM_MK_TO_COMP						(1 << 0)	// ���� ��������������� ����� � ������ ���������� ������
#define WIM_INPUT_SETTING					(1 << 1)	// ���� ������ �������� �� �����
#define WIM_FLASH_TO_COMP					(1 << 2)	// ������� ����� �������� ����������� ������ � flash ������ �� ����
#define WIM_ENCODER_OFF						(1 << 3)	// ������� �������� (1-����, 0-���)

#define WIM_MODE_USB							(1 << 4)	// ������� ��������� USB
#define WIM_MODE_UART							(1 << 5)	// ������� ��������� UART
#define WIM_MODE_FLASH_WRITE			(1 << 6)	// ����� ������ �� ����
#define WIM_MODE_FLASH_ERASE			(1 << 7)	// ����� �������� ����
#define WIM_FIFO_FILLED						(1 << 8)	// FIFO ���������
//#define WIM_GPS_TO_COMP							(1 << 4)	// 
//#define WIM_ACC_NORM_G						(1 << 16)


extern uint32_t WIM_task; 			// ���� ���������� ���������
extern volatile int count_overflow;		// ������� ���������� ������������ �������� �������� CNT ������� TIM1 (� �����)
extern int Chan1_overflow_0;
extern int Chan1_overflow_00;
extern int64_t Delta_TIM_CNT;

void TIM1_CC_IRQHandler(void);						// ���������� �� ������� �������� ������� 1 � 3
void TIM1_UP_TIM10_IRQHandler(void); 			// ���������� �� ������������ �������� �������� CNT ������� 1

void TIM8_CC_IRQHandler(void);						// ���������� �� ������� �������� ������� 1 � 3
void TIM8_UP_TIM13_IRQHandler(void);

void Processing_TIM1_chan1 (int64_t counter_CNT_chan_1, int64_t counter_CNT_chan_1_from_overflow);					// ��������� ������� 1 ������ 1
void Processing_TIM1_chan3 (int64_t counter_CNT_chan_3, int64_t counter_CNT_chan_3_from_overflow);					// ��������� ������� 1 ������ 3

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
