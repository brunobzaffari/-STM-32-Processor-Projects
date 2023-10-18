/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "time.h"
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

//PARA O USO DA UART
#define NO_LCD 1
#define NA_SERIAL 2

//RELES
#define RELE1_0	GPIOA->BRR =  1 << 0
#define RELE1_1	GPIOA->BSRR = 1 << 0
#define RELE2_0	GPIOB->BRR =  1 << 1
#define RELE2_1	GPIOB->BSRR = 1 << 1

//PARA INTERAGIR COM AS TECLAS DO DISPLAY LCD
#define PRES_DIR   0 // NÃO USADO
#define PRES_INC   1 // INC
#define PRES_DEC   2 // DEC
#define PRES_ESQ   3 // NÃO USADO
#define PRES_ENTER 4 // ENTER
#define PRES_NONE  5 // NENHUM
//#define PA0 (GPIOB->IDR & (1<<0)) // PA0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Tabela com os charsespeciais
const unsigned char tabchar[16] = { 0x00, 0x0e, 0x19, 0x15, 0x11, 0x0e, 0x00, 0x00, 0x00, 0x0e, 0x11, 0x15, 0x15, 0x0e, 0x04, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */


//-----FUNCOES CRIADAS-----//

void mostrahoras(void);

void lcd_backlight (uint8_t light);
int botao_pres(uint16_t a0_in);

void print_setup(void);
void print_botao(int botao_pres);
int rele1(void);
int rele2on(void);
int timercount(int *timer);
void c(int rele2on);
void l(int rele1);
int confighorad(void);
int configtimer(void);
int confighoras(int *botao_pres);// retorna controle
int sethora(int *hora, int botao_pres);
int setmins(int *mins, int botao_pres);
void le_botao(uint8_t a0_in, int *botao);
//-------------------------------------------------------//
//-------FUNCOES PADRAO PARA O FUNCIONAMENTO DO LCD------//
void udelay (void);
void delayus (int tempo);
void lcd_wrcom4 (uint8_t com4);
void lcd_wrcom(uint8_t com);
void lcd_wrchar(char ch);
void lcd_init(uint8_t cursor);
void lcd_wrstr(char *str);
void lcd_wr2dig(uint8_t valor);
void lcd_senddata(uint8_t data);
void lcd_clear(void);
void lcd_progchar(void);
void lcd_goto(uint8_t x, uint8_t y);
int __io_putschar(int ch);
//int fputc(int ch, FILE * f);
//-------------------------------------------------------//

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//variaveis globais
int horad=0, timer=0, hora=0, min=0, seg=0;
int h=0, m=0, s=0;
RTC_TimeTypeDef relogio;

//Implementacao da funcao printf no CubeIDE
int __io_putchar(int ch)
{
   if(ch != '\n') lcd_wrchar(ch);
   return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
	int horadset=0, timerset=0, horasset=0;
	uint16_t a0_in;
	int botao_pres;
	//int kt;
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(150);
	lcd_init(CURSOR_OFF);
	lcd_progchar();
	lcd_goto(0,0);
	printf("Starting at(\n");

	//sethora(&hora, &botao_pres), setmins(&mins, &botao_pres) e
	//setmins(&mins, &botao_pres) dentro da confighoras(&horaset,&botao_pres);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(horadset==0 ||timerset==0|| horasset==0){
    	HAL_ADC_Init(&hadc);
    	HAL_ADC_Start(&hadc);
    	HAL_ADC_PollForConversion(&hadc, 1);
    	a0_in = HAL_ADC_GetValue(&hadc);
    	HAL_ADC_Stop(&hadc);
    	// LE OS BOTOES
    	le_botao(&a0_in, &botao_pres);
    	// Seja ja configurou horad configura timer
    	if(horadset==0){
    		confighorad();
    		horadset=sethora(&horad,botao_pres);}
    	// Seja ja configurou o timer configura as horas
    	else if(timerset==0){
    		configtimer();
    		timerset=setmins(&timer, botao_pres);}
    	// Se ja configurou as horas quer dizer que:
    	// horadset=1 e timerset=1 e horasset = 1
    	else if(horasset==0){
    		horasset=confighoras(&botao_pres);}
    	HAL_Delay(1);}

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
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

//---------------------FUNCOES CRIADAS------------------///


//sethora confconfigura hora e horad
//conta tempo pressionado, especifico de 0-23.
//sethora confconfigura hora e horad
//conta tempo pressionado, especifico de 0-23.
void le_botao(uint8_t a0_in, int *botao){
	//OS VALORES APROXIMADOS FORAM 0, 572, 1500, 2470, 3775
	if (a0_in >= 4095)                     *botao==PRES_NONE;// NENHUM/NAO APERTA
	else if(a0_in <=500)                   *botao==PRES_DIR;// DIREITA
	else if((a0_in >=500)&&(a0_in <=700))  *botao==PRES_INC;// INC
	else if((a0_in >=1500)&&(a0_in <=2000))*botao==PRES_DEC;// DEC
	else if((a0_in >=2400)&&(a0_in <=2700))*botao==PRES_ESQ;// ESQUERDA
	else if((a0_in >= 3600)&&(a0_in <4002))*botao==PRES_ENTER;// ENTER
	else                                   *botao==PRES_NONE;  // DEFALT NENHUM/NAO APERTA
 }

int sethora(int *hora, int botao_pres){
	int kt;
	if (botao_pres==PRES_ENTER){
		lcd_clear();
		lcd_goto(1,0);
		printf("ENTER\n");
		return 1;}

	while(botao_pres==PRES_INC){
		// para o usuario saber
		lcd_goto(1,6);
		printf("INC  \n");
		kt=0;
		// ciclos de contagem para a botao
		while(kt<150){
		HAL_Delay(1);
		kt++;}

		*hora=*hora+1;
		// se atinge 23 vira 0
		if(*hora>23) *hora=0;}

	while(botao_pres==PRES_DEC){
		// para o usuario saber
		lcd_goto(1,6);
		printf("DEC \n");
		kt=0;
		// ciclos de contagem para a botao
		while(kt<150){
		HAL_Delay(1);
		kt++;}

		*hora=*hora-1;
		// se atinge 0 vira 23
		if(*hora<0) *hora=23;}
	return 0;
}




//setmins conf minutos, segundos e timer
//conta tempo pressionado, especifico de 0-59.
int setmins(int *mins, int botao_pres){
	int kt;
	if (botao_pres==PRES_ENTER){
		lcd_clear();
		lcd_goto(0,0);
		printf("ENTER\n");
		return 1;}
	while(botao_pres==PRES_INC){
		// para o usuario saber
		lcd_goto(1,6);
		printf("INC  \n");

		kt=0;
		// ciclos de contagem para a botao
		while(kt>150){
		HAL_Delay(1);
		kt++;}

		*mins=*mins+1; // INC
		// se atinge 59 vira 0
		if(*mins>59) *mins=0;}

	while(botao_pres==PRES_DEC){
		// para o usuario saber
		lcd_goto(1,6);
		printf("DEC \n");
		kt=0;
		// ciclos de contagem para a botao
		while(kt>150){
		HAL_Delay(1);
		kt++;}

		*mins=*mins-1; // DEC
		// se atinge 0 vira 23
		if(*mins<0) *mins=23;}
	return 0;
}

int confighorad(void){//configura hora inicial
 lcd_goto(0,0);
 printf("Starting at(\n");
 lcd_wrchar(0);
 printf("):\n");
 lcd_goto(1,0);
 printf("%02d\n", horad);
 //*controle=sethora(&horad, &botao_pres);
 return 0;
}

int configtimer(void){//configura timer
 lcd_goto(0,0);
 // para o usuario informar duracao inicial do timer
 printf("Timer( \n");
 lcd_wrchar(1);
 printf("):\n");
 lcd_goto(1,0);
 printf("%02d\n", timer);
 //*controle = setmins(&timer, &botao_pres);
 return 0;
}

int confighoras(int *botao_pres){
	lcd_goto(0,0);

	printf("H:M:S \n");
	lcd_goto(0,0);
	printf("%02d:%02d:%02d\n", hora, min, seg);
	//configura horas inicial
	//encadeameto de ifs para controlar a selecao em sequencia
	//de hora, min e seg
	if(h==0)h=sethora(&hora, *botao_pres);
	else if(m==0)m=setmins(&min, *botao_pres);
	else if(s==0)s=setmins(&seg, *botao_pres);
	else if(s==1){
	relogio.Hours = hora;
	relogio.Minutes = min;
	relogio.Seconds = seg;
	HAL_RTC_SetTime(&hrtc, &relogio, RTC_FORMAT_BIN);
	return 1;
	}
	else return 0;
}

 void print_setup(void){
 	lcd_goto(0,0);
 	l(rele1());
 	lcd_goto(1,0);
 	c(rele2on());
 	lcd_goto(0,7);
 	lcd_wrchar(0);
 	printf("%02d   \n", horad);
 	lcd_wrchar(1);
 	printf("%02d   \n", timer);
 	lcd_goto(1,7);
 	//mostrahoras();
 	printf("%02d:%02d:%02d\n", hora, min, seg);
 }

//Funcao para imprimir C['x'] ou c['.']
void c(int rele2on){
	if(rele2on==1)printf("C[X]\n");
	else if(rele2on==0) printf("C[.]\n");
}
//Ferifica se o timer pode comecar a contagem
int rele2on(void){
if((hora==horad)&&(min==00)&&(seg==00)){
	// configura relé2 como nivel logico alto
	RELE2_1;
	return 1;
}
else return 0;
}

//Funcao para imprimir L['x'] ou L['.']
void l(int rele1){
	if(rele1==1) printf("L[x]\n");
	else if(rele1==0) printf("L[.]\n");
}
//Define comportamento ditado pelo relé 1
int rele1(void){
if((hora>18)&&((hora<=22)&&(min<=59)&&(seg<=59))){
	RELE1_1;
	lcd_backlight(1);
	return 1;}

	else {
	RELE1_0;
	lcd_backlight(0);
	return 0;}
}
void lcd_backlight(uint8_t light){
	if(light == 0){
		GPIOB -> BRR = 1<<3;
	} else {
		GPIOB -> BSRR= 1<<3;
	}
}
void mostrahoras(void){
	HAL_RTC_GetTime(&hrtc, &relogio, RTC_FORMAT_BIN);

	hora = relogio.Hours;
	min  = relogio.Minutes;
	seg  = relogio.Seconds;

	printf("%02d:%02d:%02d\n", hora, min, seg);
}
//---------------------------------------------------------------//
//-------------FUNCOES LCD---------------------------------------//
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

void lcd_progchar(void)
{
	int i;
	lcd_wrcom(0x40);
	for(i=0;i<16;i++)
	{
		lcd_wrchar(tabchar[i]);
	}
	lcd_wrcom(0x40);//0x80?
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