#include "stm32f4xx_hal.h"
#include "main.h"

#define TIMER_OFF()     TIM6->CR1 &= ~TIM_CR1_CEN
#define TIMER_ON()      TIM6->CR1 |= TIM_CR1_CEN;

void DAC_Init(void);
void TIM6_Init(void);