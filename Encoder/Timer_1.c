#include "stm32f4xx_tim.h"
#include "Timer_1.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"


static TIM_ICInitTypeDef  TIM_ICInitStructure;

int TIM1_Init(void)
{
  // TIM1 Configuration 
  TIM1_Config();

  /* TIM1 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM1 CH2 pin (PE.11)  
     The Rising edge is used as active edge,
     The TIM1 CCR2 is used to compute the frequency value 
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM1, ENABLE);

  /* Enable the CC1 Interrupt Request */
//  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
}

/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void TIM1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOE clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  /* TIM1 channel 1 pin (PE.11) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  
  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		// 2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void INTTIM_Config(void)
{
	NVIC_InitTypeDef nvic_struct;
	TIM_TimeBaseInitTypeDef tim_struct;
	nvic_struct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_struct.NVIC_IRQChannelSubPriority = 0x01;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	tim_struct.TIM_Period = 65536 - 1;
	tim_struct.TIM_Prescaler = 1 - 1;
	tim_struct.TIM_ClockDivision = 0;
	tim_struct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &tim_struct);
//	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM1, ENABLE);
}

int TIM1_DeInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM1 clock disable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);

  /* GPIOE clock disable */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, DISABLE);
    
  /* Disable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		// 2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM disable counter */
  TIM_Cmd(TIM1, DISABLE);

  /* Disable the CC1 Interrupt Request */
  TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);

}

void INTTIM_Deinit(void)
{
	NVIC_InitTypeDef nvic_struct;
	TIM_TimeBaseInitTypeDef tim_struct;
	nvic_struct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_struct.NVIC_IRQChannelSubPriority = 1;
	nvic_struct.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&nvic_struct);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
	tim_struct.TIM_Period = 65536 - 1;
	tim_struct.TIM_Prescaler = 1 - 1;
	tim_struct.TIM_ClockDivision = 0;
	tim_struct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &tim_struct);
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM1, DISABLE);
}
