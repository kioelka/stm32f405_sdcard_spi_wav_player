#include "periphery_init.h"

extern char buffer[2][BUFFER_SIZE];
extern FIL file;
extern uint8_t curBufIdx ;
extern uint16_t curBufOffset ;
extern uint8_t wavReadFlag ;
extern uint32_t wavDataSize ;
extern uint32_t curWavIdx ;
extern uint8_t stopFlag;


void DAC_Init(){
    //буду использовать DAC1 (нога PA4)
    
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;      // вкл. тактирование ЦАП
    DAC->CR |= DAC_CR_EN1;          // вкл. ЦАП 1
}

void TIM6_Init(void)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;             // тактирование таймера
        
        TIM6->PSC = 1;                             // предделитель
        TIM6->ARR = 5250/8-1;                  // переполнение для Частоты 32 Khz
        
        TIM6->DIER |= TIM_DIER_UIE;             // прерывание по переполнению
        //TIM6->DIER |= TIM_DIER_UDE;             // влк. запуск ПДП
        
        TIM6->CR2 |= TIM_CR2_MMS_1;     // ЦАП будет запускаться по переполнению
        //TIM6->CR1 |= TIM_CR1_CEN;               // запуск счета  
        NVIC_SetPriority(TIM6_DAC_IRQn,0);
        NVIC_SetPriority(OTG_FS_IRQn,1);
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

void TIM6_DAC_IRQHandler(){
    
    DAC->DHR8R1 = buffer[0][curBufOffset++];
    curWavIdx++;
    
    if (curWavIdx >= wavDataSize)
        {
            TIMER_OFF();
            stopFlag = 1;
        }
    else
        {
            if (curBufOffset == BUFFER_SIZE)
                {
                    curBufOffset = 0;
                    if (curBufIdx == 0)
                        curBufIdx = 1;
                    else
                        curBufIdx = 0;
                    wavReadFlag = 1;
                }
        }
    TIM6->SR = 0;
}

void DMA_Init(){
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;       // подаю такты на DMA1
    DMA1_Stream5->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE); // 8bit wide
    DMA1_Stream5->CR |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC; // чтение с памяти и инкремент памяти
    DMA1_Stream5->PAR = (uint32_t) &DAC->DHR8R1;
    DMA1_Stream5->M0AR = (uint32_t) &buffer;
    DMA1_Stream5->NDTR = BUFFER_SIZE;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
}