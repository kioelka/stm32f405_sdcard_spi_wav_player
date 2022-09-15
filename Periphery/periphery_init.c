#include "periphery_init.h"

extern char buffer[BUFFER_SIZE];
extern short i;

void DAC_Init(){
    //буду использовать DAC1 (нога PA4)
    
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;      // вкл. тактирование ЦАП
    DAC->CR |= DAC_CR_EN1;          // вкл. ЦАП 1
}

void TIM6_Init(void)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;             // тактирование таймера
        
        TIM6->PSC = 0;                             // предделитель
        TIM6->ARR = 5249;                  // переполнение для Частоты 32 Khz   
        
        TIM6->DIER |= TIM_DIER_UIE;             // прерывание по переполнению
        TIM6->DIER |= TIM_DIER_UDE;             // влк. запуск ПДП
        
        TIM6->CR2 |= TIM_CR2_MMS_1;     // ЦАП будет запускаться по переполнению
        TIM6->CR1 |= TIM_CR1_CEN;               // запуск счета  
        NVIC_SetPriority(TIM6_DAC_IRQn,0);
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

void TIM6_DAC_IRQHandler(){
    DAC->DHR8R1 = buffer[i++];
    if (i>=BUFFER_SIZE)
        {
            i = 0;
            TIMER_OFF();
        }
    TIM6->SR = 0;
}