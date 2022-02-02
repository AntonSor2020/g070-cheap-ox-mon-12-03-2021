/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned int const NBuf=512;
	unsigned int const nfilt=8;//чизло значений для фильтрации
	unsigned int const nminmax=64;//чизло значений для определения бывшего максимума и минимума
	unsigned int const deadinterval=200;//защитный интервал - после него считаем новый максимум (2,5 Гц - 150 ударов в минуту и пополам)

	unsigned int const PorogRatioMax=70;//указание порога срабатывания в + как 70% от максимума (между средним и максимумом)
	unsigned int const PorogRatioMin=30;//указание порога срабатывания в - как 30% от максимума  (между средним и минимумом)

unsigned int const FingerOutRed=60000;//порог, когда палец не вставлен для красного
unsigned int const FingerOutIR=60000;//порог, когда палец не вставлен для инфракрасного
unsigned int const NotFingerRed=5000;//порог, когда вместо пальца непроницаемый для красного
unsigned int const NotFingerIR=5000;//порог, когда вместо пальца непроницаемый для инфракрасного

unsigned int const BeginTimePulce=1500;//количество миллисекунд для раскачки - чтобы прошел первый полупериод пульса

char txt[255];
int lv=0;
int iii=0;

unsigned int adcbufred[NBuf];//буфер АЦП красного
unsigned int adcbufir[NBuf];//буфер АЦП инфракрасного
unsigned int redfilter[nfilt+1];
unsigned int irfilter[nfilt+1];//массив для фильтрации значений методом скользящего среднего - либо цифровым фильтром


int ErrorFlag=0;//флаг ошибки 0 - норма, 1 - нет пальца на красном канале, 2 - нет пальца на ИК канале, 4 - низкий сигнал красного, 8 - низкий сигнал ИК, 16 - время не достигнуто
char flg=0;//флаг
char flg2=0;//флаг

unsigned int indi=0;//счетчик проходов для индикации Берем такты 0,1,2 - индикация, 3 - АЦП, 4,5,6 - индикация, 7 - АЦП, итого 8 тактов, частота 800 Гц
unsigned int redtemp, irtemp,redmean,irmean,predred,predir;
char trigmax=1;//защелка максимума 0 - порог не пройден, 1 - порог пройден, считаем новый максимум, сброс в пороге минимуме, 1 - чтобы сразу начал считать
char trigmin=1;//защелка минимума 0 - порог не пройден, 1 - порог пройден, считаем новый минимум, сброс в пороге максимуме
int predfiltered=0;//флаг начала фильтрации

unsigned int redmax=0;
unsigned int redmin=65535;
unsigned int irmax=0;
unsigned int irmin=65535;//максимум и минимум красного и инфракрасного, считаемые в текущий момент
unsigned int redmaxpred=0;
unsigned int redminpred=65535;
unsigned int irmaxpred=0;
unsigned int irminpred=65535;//максимум и минимум красного и инфракрасного, посчитанные в предыдущем периоде

int redporogmax=0;//порог для триггера определения пульса по красному светодиоду
int redporogmin=65535;

unsigned int ttek=0;//текущее время
unsigned int tbeg=0;//время начала периода - когда включили или нажали кнопку или вставили палец
	
unsigned int tredmax=0;
unsigned int tpredredmax=0;//время, когда был зафиксирован максимум и предыдущий максимум - по systick
unsigned int tirmax=0;
unsigned int tpredirmax=0;//время, когда был зафиксирован максимум и предыдущий максимум - по systick
unsigned int tredmin=0;
unsigned int tpredredmin=0;//время, когда был зафиксирован максимум и предыдущий максимум - по systick
unsigned int tirmin=0;
unsigned int tpredirmin=0;//время, когда был зафиксирован максимум и предыдущий максимум - по systick

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim15)//заводим колбек на таймер
	{//у нас частота опроса АЦП 100 Гц, и показ трех цифр индикатора на один опрос. Берем такты 0,1,2 - индикация, 3 - АЦП, 4,5,6 - индикация, 7 - АЦП, итого 8 тактов, частота 800 Гц
		//при частоте 64 МГц коэффициент деления 80 000 = 10 предделитель (9+1), 8000 делитель (7999+1)
		//счетчик проходов
	if((indi==3)|(indi==7)){//опрашиваем АЦП
		//выключаем все индикаторы
				HAL_GPIO_WritePin(SegA_GPIO_Port,SegA_Pin,0);
				HAL_GPIO_WritePin(SegB_GPIO_Port,SegB_Pin,0);
				HAL_GPIO_WritePin(SegC_GPIO_Port,SegC_Pin,0);
				HAL_GPIO_WritePin(SegD_GPIO_Port,SegD_Pin,0);
				HAL_GPIO_WritePin(SegE_GPIO_Port,SegE_Pin,0);
				HAL_GPIO_WritePin(SegF_GPIO_Port,SegF_Pin,0);
				HAL_GPIO_WritePin(SegG_GPIO_Port,SegG_Pin,0);
				HAL_GPIO_WritePin(SegDP_GPIO_Port,SegDP_Pin,0);
		
		
			HAL_GPIO_WritePin(D_1_GPIO_Port,D_1_Pin,0);	
			HAL_GPIO_WritePin(D_2_GPIO_Port,D_2_Pin,0);
			HAL_GPIO_WritePin(D_3_GPIO_Port,D_3_Pin,0);
			HAL_GPIO_WritePin(D_4_GPIO_Port,D_4_Pin,0);
			HAL_GPIO_WritePin(D_5_GPIO_Port,D_5_Pin,0);
			HAL_GPIO_WritePin(D_6_GPIO_Port,D_6_Pin,0);
		
		//включаем красный светодиод и считываем красный канал
					HAL_GPIO_WritePin(RedLed_GPIO_Port,RedLed_Pin,1);//Врубаем красный светодиод

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1);
		redtemp=HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		HAL_GPIO_WritePin(RedLed_GPIO_Port,RedLed_Pin,0);//Вырубаем красный светодиод
		//включаем инфракрасный светодиод и считываем инфракрасный канал
		HAL_GPIO_WritePin(IRLed_GPIO_Port,IRLed_Pin,1);//включаем инфракрасный
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1);
		irtemp=HAL_ADC_GetValue(&hadc1);
  	HAL_ADC_Stop(&hadc1);	
		HAL_GPIO_WritePin(IRLed_GPIO_Port,IRLed_Pin,0);//выключаем инфракрасный
		//пока без фильтрации - для пробы. позже вставим из старого кода
		
	//фильтруем методом скользящего среднего - или вкорячиваем сюда цифровой фильтр потом
	
//добавляем в конец
redmean=redfilter[nfilt];
irmean=irfilter[nfilt];
for(unsigned int jjj=nfilt;jjj>0;jjj--){
       redfilter[jjj] = redfilter[jjj-1];
	redmean+=redfilter[jjj-1];
       irfilter[jjj] = irfilter[jjj-1];
irmean+=irfilter[jjj-1];	
}
redfilter[0]=redtemp;
irfilter[0]=irtemp;

//проверяем, заполнены ли все значения
if (predfiltered<nfilt){predfiltered++;}else
{//фильтруем и показываем миру

redtemp+=redmean;
irtemp+=irmean;	

redtemp/=(nfilt+1);
irtemp/=(nfilt+1);	
		
		
		
	ttek=HAL_GetTick();//обновляем время	

	
	
	
	//проверяем, вставлен ли палец - когда не вставлен, уровень очень большой
	//при ошибке обнуляем время
	if(redtemp>FingerOutRed){ErrorFlag|=1;} else {ErrorFlag&=~1;};
	if(irtemp>FingerOutIR){ErrorFlag|=2;} else {ErrorFlag&=~2;}; 
	if(redtemp<NotFingerRed){ErrorFlag|=4;} else {ErrorFlag&=~4;};
	if(irtemp<NotFingerIR){ErrorFlag|=8;} else {ErrorFlag&=~8;};



if (ErrorFlag==0){	
		
		
		
		
		//продолжаем
	
	
		//обновляем максимумы и минимумы
		if (redtemp>redmax){redmax=redtemp;};
		if (redtemp<redmin){redmin=redtemp;};
		
		if(irtemp>irmax){irmax=irtemp;tirmax=ttek;};
		if(irtemp<irmin){irmin=irtemp;tirmin=ttek;};
		
		//теперь смотрим, прошел ли хоть один период пульса по времени - то есть, натикало ли с момемта включения или момента нажатия кнопки больше 1,5 секунд
		if((ttek-tbeg)>BeginTimePulce){//проверяем, что там обновилось и начинаем отсчитывать пульс - время между максимумами и минимумами
			//пульс проверяем по красному светодиоду - смотрим, когда он пересекает 75% от максимума
			//делим расстояние между максимумом и минимумом и умножаем на 75%
			redporogmax=redminpred+PorogRatioMax*(redmaxpred-redminpred)/100;//так как порог в процентах
			redporogmin=redminpred+PorogRatioMin*(redmaxpred-redminpred)/100;
			//ловим пересечение
			if((redtemp>redporogmax)&(predred<=redporogmax)){//ага, поймали порог в плюс - засекаем время
				tpredredmax=tredmax;
				tredmax=ttek;
				//обнуляем порог максимума - ищем новый следующего периода
				redmaxpred=redmax;
				redmax=0;
				} else if((redtemp<redporogmin)&(predred>=redporogmin)){//ага, поймали порог в минус - засекаем время
					tpredredmin=tredmin;
					tredmin=ttek;
					//обнуляем порог минимума - ищем новый следующего периода 
					redminpred=redmin;
					redmin=65535;
				};
				
		
		
		
		
		
		
			adcbufred[iii]=redtemp;//заносим в массив
		adcbufir[iii]=irtemp;


	iii++;
	if(iii>NBuf){iii=0;flg2=1;};
		}
			
			
		
		
		} else {//если ошибка
		tbeg=ttek;//обнуляем время
			iii=0;//обнуляем массив
					redminpred=redmin;
			redmaxpred=redmax;
			
			
			redporogmax=0;
		redporogmin=65535;
		redmax=0;
		redmin=65535;
		irmax=0;
		irmin=65535;

	}
		predred=redtemp;//предыдущие значения
	predir=irtemp;
}	
		}else{//индикация
			HAL_GPIO_WritePin(SegDP_GPIO_Port,SegDP_Pin,0);
			HAL_GPIO_WritePin(D_1_GPIO_Port,D_1_Pin,1);
	}
		
		
indi++;
if(indi>7){indi=0;};


}
	

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//рапортуем о включении
	lv=snprintf(txt,255,"\r\nLED Pulce Ox Meter\r\n");
	HAL_UART_Transmit(&huart2,txt,lv,100);
	//включаем усилитель
HAL_GPIO_WritePin(AmplPower_GPIO_Port,AmplPower_Pin,1);
//	HAL_GPIO_WritePin(AmplPower_GPIO_Port,AmplPower_Pin,0);
	//ждем чутка
	HAL_Delay(100);
	tbeg=HAL_GetTick();//начало отсчета
HAL_TIM_Base_Start_IT(&htim15);//стартуем
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
			//отчитываемся для теста
		lv=snprintf(txt,255,"\r\nred %d ir %d time %d red porog max %d min %d t red max %d pred %d diff %d min %d pred %d diff %d error %d\r\n",redtemp,irtemp,ttek,redporogmax, redporogmin,tredmax,tpredredmax,tredmax-tpredredmax,tredmin,tpredredmin,tredmin-tpredredmin, ErrorFlag);
	HAL_UART_Transmit(&huart2,txt,lv,100);
		
		if(ErrorFlag&1){
	lv=snprintf(txt,255,"\r\nFinger Out Red\r\n");
	HAL_UART_Transmit(&huart2,txt,lv,100);
	};
 if(ErrorFlag&2){
		lv=snprintf(txt,255,"\r\nFinger Out IR\r\n");
	HAL_UART_Transmit(&huart2,txt,lv,100);
	};
 if(ErrorFlag&4){
	lv=snprintf(txt,255,"\r\nSignal Red Too Low\r\n");
	HAL_UART_Transmit(&huart2,txt,lv,100);
	};
 if(ErrorFlag&8){
		lv=snprintf(txt,255,"\r\nSignal IR Too Low\r\n");
	HAL_UART_Transmit(&huart2,txt,lv,100);
	};
		
	if (flg2){
	flg2=0;		
	lv=snprintf(txt,255,"\n\rRed;");
HAL_UART_Transmit(&huart2,txt,lv,100);
	for(int iii=0;iii<NBuf;iii++){
lv=snprintf(txt,255," %d;",adcbufred[iii]);
		HAL_UART_Transmit(&huart2,txt,lv,100);
	}
		lv=snprintf(txt,255,"\n\rIR;");
HAL_UART_Transmit(&huart2,txt,lv,100);
	for(int iii=0;iii<NBuf;iii++){
lv=snprintf(txt,255," %d;",adcbufir[iii]);
		HAL_UART_Transmit(&huart2,txt,lv,100);
	}	
}
	HAL_Delay(500);
		HAL_IWDG_Refresh(&hiwdg);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 32;//частота 64 МГц
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  //hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 9;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 7999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SegA_Pin|SegF_Pin|SegB_Pin|SegG_Pin
                          |SegE_Pin|D_1_Pin|D_3_Pin|D_4_Pin
                          |D_5_Pin|D_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SegC_Pin|SegDP_Pin|SegD_Pin|RedLed_Pin
                          |IRLed_Pin|AmplPower_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D_2_GPIO_Port, D_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SegA_Pin SegF_Pin SegB_Pin SegG_Pin
                           SegE_Pin D_1_Pin D_3_Pin D_4_Pin
                           D_5_Pin D_6_Pin */
  GPIO_InitStruct.Pin = SegA_Pin|SegF_Pin|SegB_Pin|SegG_Pin
                          |SegE_Pin|D_1_Pin|D_3_Pin|D_4_Pin
                          |D_5_Pin|D_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SegC_Pin SegDP_Pin SegD_Pin RedLed_Pin
                           IRLed_Pin AmplPower_Pin */
  GPIO_InitStruct.Pin = SegC_Pin|SegDP_Pin|SegD_Pin|RedLed_Pin
                          |IRLed_Pin|AmplPower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D_2_Pin */
  GPIO_InitStruct.Pin = D_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D_2_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
