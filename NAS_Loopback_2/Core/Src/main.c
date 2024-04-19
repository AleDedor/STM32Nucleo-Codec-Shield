/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "TLV320AIC3101_Codec.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIGH 1
#define LOW 0
#define NUM_LEDS 5
#define TIMEOUT 500
#define BUFF_SIZE 50
#define RESET_TIME 50 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t TIM3_ISR_FLAG = 0;
uint16_t LED_PIN[NUM_LEDS]={RLED1_Pin, RLED2_Pin, YLED1_Pin, YLED2_Pin, GLED1_Pin};
Codec codec;
// buffer of sound data where odd indexes are left channel, even are right one
uint16_t rx_data[BUFF_SIZE];
uint16_t tx_data[BUFF_SIZE];
static volatile uint16_t* inBufPtr;
static volatile uint16_t* outBufPtr = &tx_data[0];
char buff[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
static void Led_Clear();
//static void Codec_Setup();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Led_Clear(){
	HAL_GPIO_WritePin(GPIOA, RLED1_Pin|RLED2_Pin|YLED1_Pin|YLED2_Pin|GLED1_Pin, GPIO_PIN_RESET);
}

/* 2 DMA streams are used, 1 RX ,1 TX */
/* move half of the receiving DMA into the half of the transmitting DMA */
void process_half(){

	uint16_t volume = 0;

	for(uint8_t n=0 ; n < (BUFF_SIZE/2) - 1; n++){
		//LEFT
		*(outBufPtr+n)=*(inBufPtr+n); //tx_data --> rx_data
		//RIGHT
		//outBufPtr[n+1]=inBufPtr[n+1];

		//VU METER FATTO MALE SOLO PER PROVARE
		volume = (*(inBufPtr+n));
		Led_Clear();
		if(volume>55296){
			HAL_GPIO_WritePin(GPIOA, LED_PIN[0], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[1], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[2], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[3], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[4], HIGH);
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}else if(volume<55296 && volume>13824){
			HAL_GPIO_WritePin(GPIOA, LED_PIN[1], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[2], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[3], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[4], HIGH);
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}else if(volume<13824 && volume>4608){
			HAL_GPIO_WritePin(GPIOA, LED_PIN[2], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[3], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[4], HIGH);
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}else if(volume<4608 && volume>1536){
			HAL_GPIO_WritePin(GPIOA, LED_PIN[3], HIGH);
			HAL_GPIO_WritePin(GPIOA, LED_PIN[4], HIGH);
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}else if(volume<1536 && volume>512){
			HAL_GPIO_WritePin(GPIOA, LED_PIN[4], HIGH);
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}else if(volume<512 && volume>0){
			//HAL_GPIO_WritePin(GPIOA, LED_PIN[5], HIGH);
		}
	}
}

/* first half ready DMA callback*/
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	inBufPtr = rx_data; // pointing to rx_data of DMA
	outBufPtr = tx_data; // pointing to tx_data of DMA
	process_half(); // move data from RX to TX DMA

}
/* second half ready DMA callback*/
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	inBufPtr = (rx_data + BUFF_SIZE/2);
	outBufPtr = (tx_data + BUFF_SIZE/2);
	process_half(); // move data from RX to TX DMA
}
/*
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//HAL_I2S_Transmit_(hi2s, rx_data, BUFF_SIZE/2, 10);
	HAL_I2S_Transmit_DMA(&hi2s2, rx_data, BUFF_SIZE/2);
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	//HAL_I2S_Transmit(hi2s, (rx_data + BUFF_SIZE/2), BUFF_SIZE/2, 10);
	HAL_I2S_Transmit_DMA(&hi2s2, (rx_data + BUFF_SIZE/2), BUFF_SIZE/2);
}
*/
/*TIMER 3 used to turn on LED every second*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3)
		TIM3_ISR_FLAG = 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t led_index = 0, len = 0, reg_val = 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  /* Wait for Codec HW reset */
  HAL_Delay(RESET_TIME);


  /* Codec Setup */
  if(Codec_Init(&codec, &hi2c1) != HAL_OK){
	  while(1){
	  		  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
	  		  HAL_Delay(300);
	  }}
  else{
	  len = snprintf(buff, sizeof(buff),"Inizializzazione Codec riuscita.\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  }

  /* Check registers value to be ok */
  Codec_ReadRegister(&codec, 0x0B, &reg_val);
  len = snprintf(buff, sizeof(buff),"Registro 11: %x \r\n", reg_val);
  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  HAL_Delay(10);

  Codec_ReadRegister(&codec, 0x24, &reg_val);
  len = snprintf(buff, sizeof(buff),"Registro 36: %x \r\n", reg_val);
  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  HAL_Delay(10);

  Codec_ReadRegister(&codec, 0x5E, &reg_val);
  len = snprintf(buff, sizeof(buff),"Registro 94: %x \r\n", reg_val);
  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  HAL_Delay(10);

  Codec_ReadRegister(&codec, 0x5F, &reg_val);
  len = snprintf(buff, sizeof(buff),"Registro 95: %x \r\n", reg_val);
  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  HAL_Delay(10);

  Codec_ReadRegister(&codec, 0x60, &reg_val);
  len = snprintf(buff, sizeof(buff),"Registro 96: %x \r\n", reg_val);
  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);

  HAL_Delay(TIMEOUT);

  /* Start I2S with DMA */
  if(HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_data, rx_data, BUFF_SIZE) != HAL_OK){
	  while(1){
		  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
		  HAL_Delay(300);
	  }}
  else{
	  len = snprintf(buff, sizeof(buff),"Startato DMA per I2S\n\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)buff, len, 100);
  }


  //Start the Timer 3 to turn on LEDS
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
	  /*if(TIM3_ISR_FLAG){
		  //set new led
		  HAL_GPIO_WritePin(GPIOA, LED_PIN[led_index], HIGH);
		  led_index++;
		  if(led_index == NUM_LEDS+1){
			  led_index = 0;
			  //reset leds
			  Led_Clear();
		  }
		  TIM3_ISR_FLAG = 0;
      // OK, HPLOUT+HPROUT on, not short circuited
      // OK, HPLCOM+HPRCOM on, not short circuited
      // OK, DAC selected L2 path to high power outs + OK, not muted 

	  }*/

	  //HAL_I2S_Receive_DMA(&hi2s2, rx_data, BUFF_SIZE);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RLED1_Pin|RLED2_Pin|YLED1_Pin
                          |YLED2_Pin|GLED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RLED1_Pin RLED2_Pin YLED1_Pin
                           YLED2_Pin GLED1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RLED1_Pin|RLED2_Pin|YLED1_Pin
                          |YLED2_Pin|GLED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
