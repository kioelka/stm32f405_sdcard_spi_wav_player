#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "stm32f4xx_hal.h"

#include "fatfs_sd.h"
#include "string.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);


FATFS fs;
FIL file;
FRESULT fres;
char buffer[BUFFER_SIZE];

uint32_t dr,bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

int bufsize(char *buf){
    int i = 0;
    while(*buf++ != 0) i++;
    return i;
}
void clear_buffer(void) {
    for (int i = 0; i < BUFFER_SIZE; i++)
        buffer[i] = 0;
}

short i = 0;


int main(void)
    {
        
        /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
        HAL_Init();
        
        SystemClock_Config();
        
        /* Initialize all configured peripherals */
        
        MX_GPIO_Init();
        MX_SPI1_Init();
        MX_DMA_Init();
        MX_FATFS_Init();
        DAC_Init();
        TIM6_Init();
        
        /** mount SD */
        fres = f_mount(&fs,"",0);
        if (fres != FR_OK)
            {
                printf("error\n");
                Error_Handler();
            }
        else
            printf("SD Card connected..\n");
        
        
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
        
        SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk;
        
        DWT->CYCCNT = 0;
        
        f_getfree("",&fre_clust,&pfs);
        total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
        sprintf(buffer, "SD Card Total size: \t%lu\n",total);
        printf(buffer);
        free_space = (uint32_t)(fre_clust*pfs->csize*0.5);
        sprintf(buffer,"SD Card Free Space: \t%lu\n",free_space);
        printf(buffer);
        
        fres = f_open(&file, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
        fres = f_puts((const TCHAR*)"This data written in STM32 Code\r\n",&file);
        fres = f_close(&file);
        
        fres = f_open(&file, "test.wav", FA_READ);
        printf("Data size: %d\n",file.obj.objsize);
        UINT *p;
        
        while (i == 0 & !f_eof(&file))
            {
                if (f_read(&file,buffer,BUFFER_SIZE,p))
                    break;
                TIMER_ON();
                
            }
        fres = f_close(&file);
        printf("Cycles: %d\n",DWT->CYCCNT);
        
        MX_USB_DEVICE_Init();  
        
        while (1)
            {
            }
    }


void SystemClock_Config(void)
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
        
        __HAL_RCC_PWR_CLK_ENABLE();
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 4;
        RCC_OscInitStruct.PLL.PLLN = 168;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
            {
                Error_Handler();
            }
        /** Initializes the CPU, AHB and APB buses clocks
        */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;
        
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
            {
                Error_Handler();
            }
    }


static void MX_SPI1_Init(void)
    {
        
        hspi1.Instance = SPI1;
        hspi1.Init.Mode = SPI_MODE_MASTER;
        hspi1.Init.Direction = SPI_DIRECTION_2LINES;
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi1.Init.NSS = SPI_NSS_SOFT;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi1.Init.CRCPolynomial = 10;
        if (HAL_SPI_Init(&hspi1) != HAL_OK)
            {
                Error_Handler();
            }
        
    }

/**
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
    {
        
        /* DMA controller clock enable */
        __HAL_RCC_DMA2_CLK_ENABLE();
        
        /* DMA interrupt init */
        /* DMA2_Stream0_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        /* DMA2_Stream3_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
        
    }

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        
        /*Configure GPIO pin : SPI_CS_Pin */
        GPIO_InitStruct.Pin = SPI_CS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);
        
    }

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
    {
        __disable_irq();
        while (1)
            {
            }
    }

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
    {

    }
#endif 