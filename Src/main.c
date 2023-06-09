/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "drv_serial.h"
#include "gps.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART4_Init(void);

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
UART_HandleTypeDef uarthandl;
uint8_t str[3];

serialPort_t serial;

extern bool GPS_FIX;
int32_t GPS_coord[2] = {0,0};
uint8_t GPS_numSat = 0;
uint16_t GPS_altitude = 0, GPS_speed = 0;   // altitude in 0.1m and speed in 0.1m/s
uint16_t GPS_ground_course = 0;     // degrees * 10



#define FLASH_PAGE_COUNT 4
#define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * 1)

static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + (FLASH_PAGE_SIZE * 3); //FLASH_PAGE_COUNT - (CONFIG_SIZE / 128*1024)

int main (void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_USART4_Init();
	
	ILI9341_Init();
	ILI9341_FillScreen (BLACK);
	
//	uint32_t sector_error = 0;
//	FLASH_EraseInitTypeDef	EraseInitStruct;
//	EraseInitStruct.Banks = FLASH_BANK_1;
//	EraseInitStruct.NbSectors = 1;
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//	EraseInitStruct.Sector = FLASH_SECTOR_7;
//	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  
	__disable_irq();
//	HAL_FLASH_Unlock();
//	HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
//	HAL_FLASH_Lock();
	
//	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	
	if (*(uint32_t*)FLASH_WRITE_ADDR != 0x55667788) {
		
		
		//HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_FLASH_Unlock();
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_WRITE_ADDR, 0x55667788);		
		HAL_FLASH_Lock();
				
	}
	__enable_irq();

	HAL_UART_Receive_IT(&uarthandl, str, 1);
	
	uint32_t timer_gps_get = HAL_GetTick();
	uint32_t timer_LCD_out = HAL_GetTick();
	while(1)
	{
		static uint32_t Data;
		static char BufferText[40];
		
		Data = *(uint32_t*)FLASH_WRITE_ADDR;
		sprintf(BufferText, "0x%X  %X", FLASH_WRITE_ADDR, Data);
		
		
		
		if ((HAL_GetTick() - timer_LCD_out) > LED_TIMEOUT) {
			
			timer_LCD_out = HAL_GetTick();
			sprintf(BufferText, "%u  %u", serial.rxBufferHead, serial.rxBufferTail);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			ILI9341_DrawText (BufferText, FONT1, 10, 20, WHITE, BLACK);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);			
			
//			if ((serial.rxBufferHead - serial.rxBufferTail) > 3) {
//				ILI9341_DrawText (((char*)serial.rxBuffer)+serial.rxBufferTail, FONT1, 10, 50, GREEN, BLUE);
//				serial.rxBufferTail = (serial.rxBufferTail + 3) % serial.rx_bufferSize;
//			}
			
			sprintf(BufferText, "LAT %u  LON %u", GPS_coord[0], GPS_coord[1]);
			ILI9341_DrawText (BufferText, FONT1, 10, 40, WHITE, BLACK);
			sprintf(BufferText, "N %u  ALT %u", GPS_numSat, GPS_altitude);
			ILI9341_DrawText (BufferText, FONT1, 10, 60, WHITE, BLACK);			
			sprintf(BufferText, "SP %u  CUR %u", GPS_speed, GPS_ground_course);
			ILI9341_DrawText (BufferText, FONT1, 10, 80, WHITE, BLACK);
		
			if ((HAL_GetTick() - timer_gps_get) > GPS_TIMEOUT) {
				timer_gps_get = HAL_GetTick();						
				gpsThread();
				
				if(GPS_FIX) {
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
				} else {
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
				}
			}
		}
		
	}
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	//	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins :  */
	GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin|LD5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

//	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET);

	/* LCD PIN CONFIG*/
	HAL_GPIO_WritePin(GPIOC, LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin|LCD_LED_Pin, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET);
}

static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_USART4_Init (void)
{
	uarthandl.Instance			= UART4;
	uarthandl.Init.BaudRate		= 9600;
	uarthandl.Init.WordLength 	= UART_WORDLENGTH_8B;
	uarthandl.Init.StopBits		= UART_STOPBITS_1;
	uarthandl.Init.Parity		= UART_PARITY_NONE;
	uarthandl.Init.HwFlowCtl	= UART_HWCONTROL_NONE;
	uarthandl.Init.Mode			= UART_MODE_RX;
	
	
//	if(HAL_UART_DeInit(&uarthandl) != HAL_OK) {
//		Error_Handler();
//	}
	if(HAL_UART_Init(&uarthandl) != HAL_OK) {
		
		Error_Handler();
	}
	serialInit(&serial);	
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart4)
{
	
	HAL_UART_Receive_IT(&uarthandl, str, 1);
	
	if(huart4 == &uarthandl) {
		serial.rxBuffer[serial.rxBufferHead] = str[0];
		serial.rxBufferHead = (serial.rxBufferHead + 1) % serial.rx_bufferSize;
		
	}
}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

