
#include "stm32f10x_it.h"

float duty = 0;
extern uint16_t CCR1_Val;
extern __IO uint16_t ADCConvertedValue;
extern TIM_OCInitTypeDef  TIM_OCInitStructure;

void TIM3_IRQHandler(void)
{
	duty = (ADCConvertedValue/4096.0)*685.0;
	CCR1_Val = (uint16_t)duty + 1;
	
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
