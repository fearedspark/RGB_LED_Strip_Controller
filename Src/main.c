/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  NEO_PIXEL = 0,
  DOT_STAR
} ModeTypedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NLEDS 288
#define BUFFER_NUMBER 2
#define GetColorRotB() (GPIOA->IDR & 0x00000002)
#define GetPwrRotB() (GPIOA->IDR & 0x00000008)
#define HUE_INCREMENT 2
#define INTENSITY_INCREMENT 5

#define ABS(x) (((x)<0)?-(x):(x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int LedIntensity = 0; // 0-100
volatile int LedHue = 0;       // 0-359
volatile uint32_t DisplayWhite = 1;

// Set as a global variable to be included in memory estimation
uint32_t colors_buffer[BUFFER_NUMBER * (NLEDS + 2)];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t RGB(uint8_t R, uint8_t G, uint8_t B);
uint32_t Hue2RGB(int H, int V);
void TransmitNeopixel(uint32_t * colors, uint32_t length);
void TransmitDotstar(uint32_t * colors, uint32_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Initializes peripherals to read PB1 and return the mode
ModeTypedef ReadMode(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mode_GPIO_Port, &GPIO_InitStruct);

  if(HAL_GPIO_ReadPin(Mode_GPIO_Port, Mode_Pin) == GPIO_PIN_RESET)
    return NEO_PIXEL;
  else
    return DOT_STAR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t * colors_arrays[BUFFER_NUMBER];
  int current_buffer = 0;
  ModeTypedef led_mode = ReadMode();
  if(led_mode == DOT_STAR)
  {
    for(int i = 0; i < BUFFER_NUMBER; i++)
    {
      colors_arrays[i] = colors_buffer + i * (NLEDS + 2) + 1;
      colors_buffer[i * (NLEDS + 2)] = 0x00000000;
      colors_buffer[(i + 1) * (NLEDS + 2) - 1] = 0xFFFFFFFF;
    }
  }
  else
  {
    for(int i = 0; i < BUFFER_NUMBER; i++)
    {
      colors_arrays[i] = colors_buffer + i * NLEDS;
    }
  }
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t color = 0;
    if(DisplayWhite)
    {
      int color_intensity = (LedIntensity * 255) / 100;
      color = RGB(color_intensity, color_intensity, color_intensity);
    }
    else
      color = Hue2RGB(LedHue, LedIntensity);
    
    for(int i = 0; i < NLEDS; i++)
      colors_arrays[current_buffer][i] = color;
    if(led_mode == DOT_STAR)
      TransmitDotstar(colors_arrays[current_buffer] - 1, NLEDS + 2);
    else
      TransmitNeopixel(colors_arrays[current_buffer], NLEDS);
    current_buffer = (current_buffer + 1) % BUFFER_NUMBER;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : ColorRotA_Pin ColorButton_Pin PwrRotA_Pin PwrButton_Pin */
  GPIO_InitStruct.Pin = ColorRotA_Pin|ColorButton_Pin|PwrRotA_Pin|PwrButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ColorRotB_Pin PwrRotB_Pin */
  GPIO_InitStruct.Pin = ColorRotB_Pin|PwrRotB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Mode_Pin */
  GPIO_InitStruct.Pin = Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mode_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
uint32_t RGB(uint8_t R, uint8_t G, uint8_t B)
{
    return __REV(0xFF000000 | (uint32_t)B << 16 | (uint32_t)G << 8 | (uint32_t)R);
}

uint32_t Hue2RGB(int H, int V)
{
    // H = [0:359]
    // V = [0:100]
    if(V > 100)
        V = 100;
    if(V < 0)
        V = 0;
    H %= 360;
    if(H < 0)
      H += 360;
    int X = (V * 255 * (1 - ABS(((H / 60) % 2) - 1))) / 100;
    V = (V * 255) / 100;
    if(X > 255)
      X = 255;
    int S = H / 60;
    switch(S)
    {
        case 0:
            return RGB((uint8_t)V, (uint8_t)X, 0);
        case 1:
            return RGB((uint8_t)X, (uint8_t)V, 0);
        case 2:
            return RGB(0, (uint8_t)V, (uint8_t)X);
        case 3:
            return RGB(0, (uint8_t)X, (uint8_t)V);
        case 4:
            return RGB((uint8_t)X, 0, (uint8_t)V);
        case 5:
            return RGB((uint8_t)V, 0, (uint8_t)X);
        default:
            return RGB(0, 0, 0);
    }
}

void TransmitNeopixel(uint32_t * colors, uint32_t length)
{
}

void TransmitDotstar(uint32_t * colors, uint32_t length)
{
	HAL_StatusTypeDef spi_status;
	do
	{
		spi_status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)colors, length * 4);
	}
	while(spi_status == HAL_BUSY);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case ColorRotA_Pin:
			DisplayWhite = 0;
			if(GetColorRotB())
				LedHue += HUE_INCREMENT;
			else
				LedHue -= HUE_INCREMENT;
      LedHue %= 360;
      if(LedHue < 0)
        LedHue += 360;
			break;
		case ColorButton_Pin:
			if(DisplayWhite)
				DisplayWhite = 0;
			else
				DisplayWhite = 1;
			break;
		case PwrRotA_Pin:
			if(GetPwrRotB())
				LedIntensity += INTENSITY_INCREMENT;
			else
				LedIntensity -= INTENSITY_INCREMENT;
			if(LedIntensity > 100)
				LedIntensity = 100;
			if(LedIntensity < 0)
				LedIntensity = 0;
			break;
		case PwrButton_Pin:
			if(LedIntensity > 0)
				LedIntensity = 0;
			else
				LedIntensity = 100;
			break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
