/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Estado do Cursor
#define CURSOR_OFF 0x0C   // Apagado
#define CURSOR_ON 0x0E    // Ligado
#define CURSOR_BLINK 0x0F // Piscante

// Estado dos pinos de Controle...
#define RS_0 GPIOA -> BRR =  1<<9 //PA9
#define RS_1 GPIOA -> BSRR = 1<<9 //PA9
#define EN_0 GPIOC -> BRR =  1<<7 //PC7
#define EN_1 GPIOC -> BSRR = 1<<7 //PC7

// Estado dos pinos do Barramento do LCD...
#define D7_0 GPIOA -> BRR  = 1<<8 //PA8
#define D7_1 GPIOA -> BSRR = 1<<8 //PA8

#define D6_0 GPIOB -> BRR  = 1<<10 //PB10
#define D6_1 GPIOB -> BSRR = 1<<10 //PB10

#define D5_0 GPIOB -> BRR  = 1<<4 //PB4
#define D5_1 GPIOB -> BSRR = 1<<4 //PB4

#define D4_0 GPIOB -> BRR  = 1<<5 //PB5
#define D4_1 GPIOB -> BSRR = 1<<5 //PB5

#define NO_LCD 1
#define NA_SERIAL 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

//-------FUNCOES PADRAO PARA O FUNCIONAMENTO DO LCD------//
void udelay(void);
void delayus(int tempo);
void lcd_wrcom4 (uint8_t com4);
void lcd_wrcom(uint8_t com);
void lcd_wrchar(char ch);
void lcd_init(uint8_t cursor);
void lcd_wrstr(char *str);
void lcd_wr2dig(uint8_t valor);
void lcd_senddata(uint8_t data);
void lcd_clear(void);
void lcd_progchar(uint8_t n);
void lcd_goto(uint8_t x, uint8_t y);
int __io_putschar(int ch);
int fputc(int ch, FILE * f);
//------------------------------------------------------//
//x = aonde quer por
void tela_inicial(int x);
void mostradata(int x);
void mostrahoras(int x);
void ajuste_hora(void);
void ajuste_data(void);
void cadastra(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//HAL_UART_Receive((&huartx, uint8_t* pdata, uint16_t size, uint32_t timeout)
//HAL_UART_Transmit(&huartx, uint8_t* pdata, uint16_t size, uint32_t timeout)
//timeout>=(numerode bits)/boundrate*size
//8N1-> 10bits ->numero de bits= start+dados+pariedade+stopbit

// Variaveis globais
uint8_t dia, mes, ano, hora, min, seg;
RTC_TimeTypeDef relogio;
RTC_DateTypeDef calendario;
char user[9];//

char senha[5];

char AONDE=NO_LCD;
int erro = 0;


int __io_putchar(int ch){
	if (AONDE == NO_LCD)
	{
		if (ch != '\n') lcd_wrchar(ch);
	}
	if (AONDE == NA_SERIAL)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
	}
	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int leitura;
	float Temp, f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	uint8_t posM=0;
	int leitura1;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Init(&huart2);
	HAL_I2C_Init(&hi2c1);
	HAL_RTC_Init(&hrtc);
	HAL_RTC_WaitForSynchro(&hrtc);
	HAL_ADC_Init(&hadc);
	ADC_ChannelConfTypeDef configAD;
	lcd_init(CURSOR_OFF);

	tela_inicial(NA_SERIAL);
	tela_inicial(NO_LCD);

	/*ajuste_hora();
	ajuste_data();
	mostradata(NA_SERIAL);
	lcd_clear();
	lcd_goto(0,0);*/
	printf("Cadastre-se!\n");
	cadastra();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	configAD.Channel = ADC_CHANNEL_1;
	while(HAL_ADC_Init(&hadc) != HAL_OK) HAL_Delay(1);
	while(HAL_ADC_ConfigChannel(&hadc, &configAD) != HAL_OK) HAL_Delay(1);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	leitura1 = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);




	Temp = 0.080586*leitura1;
	f = Temp*1.8 +32;
	lcd_goto(0,9);
	printf("%02.1f%cC\n",Temp, 223);
	mostrahoras(NO_LCD);

	lcd_goto(1,9);
	printf("%03.1f%cF\n", f, 223);
	mostradata(NO_LCD);
	HAL_Delay(1000);

	//HAL_I2C_Mem_Write(&hi2c1, 0x80, posM, 4, (uint8_t*)&Temp, 4, 4);
	//if(posM>255)posM=3;
	//HAL_Delay(1000);
	 //mostrahoras(NO_LCD);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ajuste_hora(void){
	char ch=0, u, d, h, m, s=0;
	AONDE = NO_LCD;
	lcd_goto(0,0);
	printf("CONFIG. HORAS:\n");
	HAL_Delay(1500);
	lcd_clear();
	printf("Digite a hora:\n");
	AONDE = NA_SERIAL;
	printf("\rConfiguracao das horas:\r\n");

	// AJUSTE DO HORA
	do{
		ch=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a hora [00-23]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10); // eco na serial
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		u = ch-'0';
		h = 10 * d + u;
		if(h>23)printf("\rHora invalida digite novamente!\n");
	}while(h>23);


	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Horas configuradas: %d\n", h);
	HAL_Delay(1500);
	// AJUSTE DO MINUTO
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite a min:\n");
	do{
		ch=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a minutos [00-59]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		//c++;
		u = ch-'0';
		m = 10 * d + u;
		if(m>59)printf("\rMinutos invalidos digite novamente!\n");
	}while(h>59);

	AONDE = NO_LCD;
	//lcd_clear();
	lcd_goto(0,0);
	printf("Definidas\n");

	HAL_Delay(1500);
	lcd_clear();

	relogio.Hours = h;
	relogio.Minutes = m;
	relogio.Seconds = s;
	HAL_RTC_SetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
	HAL_Delay(1500);

	mostrahoras(NA_SERIAL);
	printf("\rDefinidas\n");
}
void ajuste_data(void){
	char ch=0, u, d, yy, mm, dd;

	AONDE = NO_LCD;
	lcd_goto(0,0);
	printf("CONFIG. DATA:\n");
	HAL_Delay(1500);
	lcd_clear();
	printf("Digite o ano:\n");
	AONDE = NA_SERIAL;
	printf("\rConfiguracao data:\r\n");

	// AJUSTE DO HORA
	do{
		ch=0;
		AONDE = NA_SERIAL;
		printf("\rDigite o ano [00-99]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10); // eco na serial
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		u = ch-'0';
		yy = 10 * d + u;
		if(0>99)printf("\r Ano invalido, digite novamente!\n");
	}while(yy>99);


	AONDE = NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("Ano: %d\n", yy);
	// AJUSTE DO MINUTO
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite a mes:\n");
	do{
		ch=0;
		AONDE = NA_SERIAL;
		printf("\rDigite a mes [01-12]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		//c++;
		u = ch-'0';
		mm= 10 * d + u;
		if(mm>12)printf("\r Mes invalidos, digite novamente!\n");
	}while(mm>12||mm<01);

	AONDE = NO_LCD;
	//lcd_clear();
	lcd_goto(0,0);
	printf("Mes:%d\n", mm);
	HAL_Delay(20);
	// AJUSTE DO DIA
	lcd_clear();
	lcd_goto(0,0);
	printf("Digite dia:\n");
	do{
		ch=0;
		AONDE = NA_SERIAL;
		printf("\rDigite dia [01-31]:\r\n");
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		d = ch-'0'; // Converte ASCII em DECIMAL
		do{
			erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
		} while (erro != HAL_UART_ERROR_NONE);
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
		//c++;
		u = ch-'0';
		dd= (int)10*d + u;
		if(dd>31)printf("\r Dias invalidos digite novamente!\n");
	}while(dd>31||dd<01);

	if(mm==02&&dd>28)dd=28;
	if((mm==4||mm==6||mm==9||mm==11)&& dd>30)dd=30;


	AONDE = NO_LCD;
	//lcd_clear();
	lcd_goto(0,0);
	printf("Definidas\n");
	HAL_Delay(1500);
	lcd_goto(1,0);
	printf("%02d/%02d/%02d\n", dd, mm, yy);
	HAL_Delay(3000);
	lcd_clear();

	calendario.Date = dd;
	calendario.Month = mm;
	calendario.Year = yy;
	HAL_RTC_SetDate(&hrtc, &calendario, RTC_FORMAT_BIN);
	HAL_Delay(3000);

	mostradata(NA_SERIAL);
	printf("\r%02d/%02d/%02d\r\n", dd, mm, yy);
	mostradata(NO_LCD);
}

void mostradata(int x){

	HAL_RTC_GetDate(&hrtc, &calendario, RTC_FORMAT_BIN);

	dia = calendario.Date;
	mes = calendario.Month;
	ano = calendario.Year;
	AONDE=x;
	lcd_goto(0,0);
	if(AONDE==NA_SERIAL) printf("\r%02d/%02d/%02d\r\n", dia, mes, ano);
	else printf("%02d/%02d/%02d\n", dia, mes, ano);

}
void mostrahoras(int x){
	AONDE=x;
	HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);

	hora = relogio.Hours;
	min  = relogio.Minutes;
	seg  = relogio.Seconds;

	lcd_goto(1 ,0);
	if(AONDE==NA_SERIAL) printf("\r%02d:%02d:%02d\r\n", hora, min, seg);
	else printf("%02d:%02d:%02d\n", hora, min, seg);
}

void cadastra(void){

  int i=0;
  char ch=0;
  AONDE=NA_SERIAL;
  printf("\r Diga o user 8 digitos: \r\n");
  HAL_Delay(20);

  while(i!=8){
	do{
		erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
	} while (erro != HAL_UART_ERROR_NONE);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
	user[i]=ch;
	ch=0;
	i++;
	}
    user[i]='\0';

	i=0;
	ch=0;

	printf("\rDigite a senha 4 dig: \r\n");

	while(i!=4){
	do{
		erro = HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 10);
	} while (erro != HAL_UART_ERROR_NONE);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
	senha[i]=ch;
	i++;
	ch=0;
	}
	senha[i]='\0';
	//como o ultimo foi na serial n precisa de AONDE=NA_SERIAL;
	printf("\rCadastrado:\r\nUser: %s\r\nSenha: %s\r\n", user, senha);
	//para depois os pormos na memoria
	//os traduzimos para dec
	int us = atoi(user);
	printf("\r%d\n", us);

	int se = atoi(senha);
	printf("\r%d\n", se);
	//HAL_I2C_Mem_Write(&hi2c1, 0x80, 1, 4, (uint8_t*)&us, 4, 4);
	//HAL_I2C_Mem_Write(&hi2c1, 0x80, 2, 4, (uint8_t*)&se, 4, 4);

	AONDE=NO_LCD;
	lcd_clear();
	lcd_goto(0,0);
	printf("User: %s\n  ",user);
	lcd_goto(1,0);
	printf("Senha: %s\n",senha);
	HAL_Delay(6000);
	lcd_clear();

}
void tela_inicial(int x){
	AONDE=x;
	lcd_goto(0 ,0);
	if(AONDE==NO_LCD)printf("Lab. Process\n");
	if(AONDE==NA_SERIAL)printf("\rLab. Process\n");
	lcd_goto(1 ,0);
	if(AONDE==NO_LCD)printf("Alarme Residencial\n");
	if(AONDE==NA_SERIAL)printf("\rAlarme Residencial\n");
	HAL_Delay(2700);
	lcd_clear();
}
/*
void lcd_backlight (uint8_t light){
	if(light == 0){
		GPIOB -> BRR = 1<<3;
	} else {
		GPIOB -> BSRR= 1<<3;
	}
}*/
//--------------------------------LCD----------------------------
void lcd_init(uint8_t cursor){
	lcd_wrcom4(3);
	lcd_wrcom4(3);
	lcd_wrcom4(3);
	lcd_wrcom4(2);
	lcd_wrcom(0x28);
	lcd_wrcom(cursor);
	lcd_wrcom(0x06);
	lcd_wrcom(0x01);
}

void lcd_wrcom4(uint8_t com4){
	lcd_senddata(com4); //D4...d0
	RS_0;
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}

void lcd_wrcom(uint8_t com){
	lcd_senddata(com>>4); //0000D7...D4
	RS_0;
	EN_1;
	delayus(5);
	EN_0;
	delayus(5);

	lcd_senddata(com & 0x0F); //0000D3...d0
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}

void lcd_clear(void){
	lcd_wrcom(0x01);
}

//goto para 16x2
void lcd_goto(uint8_t x, uint8_t y){
	uint8_t com = 0x80;
	if (x==0 && y<16) com = 0x80 + y;
	else if (x==1 && y<16) com = 0xC0 + y;
	else com = 0x80;
	lcd_wrcom(com);
}

void lcd_wrchar(char ch){
	lcd_senddata(ch>>4); //D7...D4
	RS_1;
	EN_1;
	delayus(5);
	EN_0;
	delayus(5);

	lcd_senddata(ch & 0x0F); //D3...D0
	RS_1;
	EN_1;
	delayus(5);
	EN_0;
	HAL_Delay(5);
}

void lcd_wrstr(char *str){
	while(*str) lcd_wrchar(*str++);
}

/*while(*str){
	lcd_wrchar(*str);
	str++;
}*/
void udelay(void){
	int tempo = 7;
	while(tempo--);
}

void delayus(int tempo){
	while(tempo--) udelay();
}

void lcd_wr2dig(uint8_t valor){
	lcd_wrchar(valor/10 + '0'); // ou +48 -> dezena
	lcd_wrchar(valor%10 + '0'); // ou +48 -> unidade
}

void lcd_senddata(uint8_t data){
	if((data & (1<<3))==0) D7_0; else D7_1;
	if((data & (1<<2))==0) D6_0; else D6_1;
	if((data & (1<<1))==0) D5_0; else D5_1;
	if((data & (1<<0))==0) D4_0; else D4_1;
}
//---------------------------------------------------------------


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
