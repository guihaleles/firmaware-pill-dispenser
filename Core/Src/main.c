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
#include <stdbool.h>
#include "queue.h"
#include "eeprom.h"
#include "stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE                     8
#define SLOTS                            8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

struct message {
	uint8_t data[RXBUFFERSIZE];
};

QUEUE_DECLARATION(my_message_queue, struct message, RXBUFFERSIZE);
QUEUE_DEFINITION(my_message_queue, struct message);
struct my_message_queue queue;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t UART1_rxBuffer[RXBUFFERSIZE] = {0};
uint8_t bluetooth_rxBuffer[RXBUFFERSIZE] = {0};
uint8_t delimiter = 0x99;

uint16_t VirtAddVarTab[SLOTS][NB_OF_VAR] = {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
                                    {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19},
                                    {0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29},
                                    {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39},
                                    {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49},
                                    {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59},
                                    {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69},
                                    {0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79}};



uint8_t configDispenserTime[SLOTS][RXBUFFERSIZE] = {0};

uint8_t position_state = 100;
float angle_state = 900;

RTC_TimeTypeDef sTime1;
RTC_DateTypeDef sDate1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Debug(uint8_t *ch, size_t numElements)
{
	HAL_UART_Transmit(&huart2,ch,numElements,10);
}

void Send_Bluettoh_Data(uint8_t *ch, size_t numElements)
{
	HAL_UART_Transmit(&huart1,ch,numElements,10);
}

void IsAlive()
{
  uint8_t data[] = {0};
  Send_Bluettoh_Data(&data,sizeof(data));
}

SetSlotPillInserted()
{  
  for(uint8_t slot = 0; slot < SLOTS; slot++)
  {
    if((EE_WriteVariable(VirtAddVarTab[0][slot],  (uint16_t)configDispenserTime[0][slot])) != HAL_OK)
      {
        uint8_t error[] = {9};
        Send_Bluettoh_Data(&error,sizeof(error));
      }
  } 
}

void GetSlotsPillsInserted(uint8_t* output)
{
  uint16_t VarDataTab[NB_OF_VAR] = {0};
  for(uint8_t i = 1; i < RXBUFFERSIZE; i++)
  {
	  if((EE_ReadVariable(VirtAddVarTab[0][i],  &VarDataTab[i])) != HAL_OK)
	  {
	      uint8_t error[] = {9};
	  }
    output[i-1] = (uint8_t) VarDataTab[i];
  }
  output[RXBUFFERSIZE-1] = 16;
}

void SetConfigDispenserTime(uint8_t* input)
{
  int slot = input[RXBUFFERSIZE-2];
  for(uint8_t i = 0; i < RXBUFFERSIZE; i++)
  {
    if((EE_WriteVariable(VirtAddVarTab[slot][i],  (uint16_t)input[i])) != HAL_OK)
      {
        uint8_t error[] = {9};
        Send_Bluettoh_Data(&error,sizeof(error));
      }
      configDispenserTime[slot][i] = (uint8_t) input[i];
  } 
}

void GetConfigDispenserTime(uint8_t* output, int slot)
{
  uint16_t VarDataTab[NB_OF_VAR] = {0};
  for(uint8_t i = 0; i < RXBUFFERSIZE; i++)
  {
	  if((EE_ReadVariable(VirtAddVarTab[slot][i],  &VarDataTab[i])) != HAL_OK)
	  {
	      uint8_t error[] = {9};
	  }
    output[i] = (uint8_t) VarDataTab[i];
  }
  output[RXBUFFERSIZE-3] = configDispenserTime[0][slot];
}

void LoadConfigDispenserTime()
{
  uint16_t VarDataTab[NB_OF_VAR] = {0};
   for(int slot = 1; slot< SLOTS; slot++)
   {
  
    for(uint8_t i = 0; i < RXBUFFERSIZE; i++)
    {
      if((EE_ReadVariable(VirtAddVarTab[slot][i],  &VarDataTab[i])) != HAL_OK)
      {
          uint8_t error[] = {9};
      }
      configDispenserTime[slot][i] = (uint8_t) VarDataTab[i];
    }
  
  }
}

void UpdateRTC()
{

	sTime1.Hours = bluetooth_rxBuffer[3];
	sTime1.Minutes = bluetooth_rxBuffer[4];
	sTime1.Seconds = bluetooth_rxBuffer[5];
	sTime1.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime1.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime1, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	sDate1.Date = bluetooth_rxBuffer[0];
	sDate1.Month = bluetooth_rxBuffer[1];
	sDate1.Year = bluetooth_rxBuffer[2];
	if (HAL_RTC_SetDate(&hrtc, &sDate1, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
}

void GetRTC(uint8_t* output)
{

  HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &sDate1, RTC_FORMAT_BCD);

  output[0] = sDate1.Date;
  output[1] = sDate1.Month;
  output[2] = sDate1.Year;
  output[3] = sTime1.Hours;
  output[4] = sTime1.Minutes;
  output[5] = sTime1.Seconds;

}

void StartDispenserRotationProcess(uint8_t* output)
{
  uint8_t position = bluetooth_rxBuffer[RXBUFFERSIZE-2];

	goToPosition(position);

  int millis = HAL_GetTick();

  SetConfigDispenserTime(bluetooth_rxBuffer);

  while(HAL_GetTick() - millis < 60000)
  {
    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
    {
      break;
    }
  }

  goHome();

  configDispenserTime[position][RXBUFFERSIZE-1] = 6; 
  SetConfigDispenserTime(configDispenserTime[position]);

	memcpy(output, bluetooth_rxBuffer, RXBUFFERSIZE * sizeof(uint8_t));
}

FinishDispenserRotationProcess(uint8_t* output)
{
  goHome();
	memcpy(output, bluetooth_rxBuffer, RXBUFFERSIZE * sizeof(uint8_t));
}

StartInsertionProcess(uint8_t* output)
{
	uint8_t position = bluetooth_rxBuffer[RXBUFFERSIZE-2];

	goToPosition(position);

	memcpy(output, bluetooth_rxBuffer, RXBUFFERSIZE * sizeof(uint8_t));
}

FinishInsetionProcess(uint8_t* output)
{
  goHome();
	memcpy(output, bluetooth_rxBuffer, RXBUFFERSIZE * sizeof(uint8_t));
}

void Command()
{
  uint8_t output[RXBUFFERSIZE] = {0};

  switch (bluetooth_rxBuffer[RXBUFFERSIZE-1])
  {
  case 1:
    IsAlive();
    break;
  case 2:
    SetConfigDispenserTime(bluetooth_rxBuffer);
    break;
  case 3:
    GetConfigDispenserTime(output, bluetooth_rxBuffer[RXBUFFERSIZE-2]);
    break;
  case 4:
    UpdateRTC();
    break;
  case 5:
    GetRTC(output);
    break;
  case 6:
	  StartDispenserRotationProcess(output);
    break;
  case 7:
	  FinishDispenserRotationProcess(output);
    break;
  case 11:
	  StartInsertionProcess(output);
    break;
  case 12:
	  FinishInsetionProcess(output);
    break;
  case 15:
	  GetPosition(output);
	  break;
  case 16:
	  GetSlotsPillsInserted(output);  
	  break;
  case 21:
    dipenserAlarm();
    break;
  default:
    break;
  }

  Send_Bluettoh_Data(&output,sizeof(output));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1){
	struct message msg;
	memcpy(msg.data, UART1_rxBuffer, RXBUFFERSIZE * sizeof(uint8_t));
	enum enqueue_result result1 = my_message_queue_enqueue(&queue, &msg);
  HAL_UART_Receive_IT(&huart1,UART1_rxBuffer,RXBUFFERSIZE);
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

	uint8_t dispenserTime[RXBUFFERSIZE] = {0};
	uint8_t rtcTime[RXBUFFERSIZE] = {0};
	GetRTC(rtcTime);

  for(int slot = 1; slot< SLOTS; slot++)
  {
    //GetConfigDispenserTime(dispenserTime,i);

    bool arrayEqual = true;

    for(uint8_t i = 0; i < RXBUFFERSIZE-2; i++)
    {
      uint8_t aux1 = configDispenserTime[slot][i];
      uint8_t aux2 = rtcTime[i];
      if(aux1 != aux2)
      {
        arrayEqual = false;
        break;
      }
    }

    if(arrayEqual)
    {
      uint8_t data[RXBUFFERSIZE] = {0};
      data[RXBUFFERSIZE-1] = 21;
      data[RXBUFFERSIZE-2] = slot;
      struct message msg;
      memcpy(msg.data, data, RXBUFFERSIZE * sizeof(uint8_t));
      enum enqueue_result result1 = my_message_queue_enqueue(&queue, &msg);

    }

  }	
}

void stepper_step_angle (float angle, int direction)
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);
  int rpm = 5;
  int presenceAux = false;
  int actualSlot = getSlotByAngle((int)angle_state);

	for (int seq=0; seq<numberofsequences; seq++)
	{


		if (direction == 0)  // for clockwise
		{
      if(!HAL_GPIO_ReadPin(GPIOA, HOME_Pin) )
      {
        angle_state = 0;
        position_state = 0;
        break;
      }
      angle_state = angle_state - 0.703125;

      if(HAL_GPIO_ReadPin(GPIOA, P1_Pin) || HAL_GPIO_ReadPin(GPIOA, P2_Pin) || HAL_GPIO_ReadPin(GPIOB, P3_Pin))
      { 
        HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
        presenceAux = true;      
      } else{
    	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
      }

      int aux = getSlotByAngle((int)angle_state);
      if(actualSlot != aux )
      {
        configDispenserTime[0][actualSlot] = presenceAux;
        SetSlotPillInserted();
        actualSlot = aux;
        presenceAux = false;
      }      

			for (int step=4; step>=0; step--)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
      angle_state = angle_state + 0.703125;
      
			for (int step=0; step<5; step++)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}   
		}
	}

}


goToPosition(int target)
{
  if(target == position_state)
  {
    return;
  }

  if(position_state != 0)
  {
    goHome();
  }

  int angle = getFinalAngleBySlot(target+1);

  stepper_step_angle(angle,1);

  position_state = target;
  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
}

startBuzzer(int value)
{
  htim3.Instance->CCR1 = value; // vary the duty cycle
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	
}

stopBuzzer()
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

dipenserAlarm()
{
  int millis = HAL_GetTick();

  uint8_t buzzerValue = 50;
  
  startBuzzer(buzzerValue);
  while(HAL_GetTick() - millis < 60000)
  {
    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
    {
      uint8_t data[RXBUFFERSIZE] = {0};
      data[RXBUFFERSIZE-1] = 6;
      data[RXBUFFERSIZE-2] = bluetooth_rxBuffer[RXBUFFERSIZE-2];
    	struct message msg;
      memcpy(msg.data, data, RXBUFFERSIZE * sizeof(uint8_t));
      enum enqueue_result result1 = my_message_queue_enqueue(&queue, &msg);

      stopBuzzer();
      return;
    }
  }
  stopBuzzer();
}

goHome()
{
  if(position_state == 0)
  {
    return;
  }

  int debounce = 0;  
  stepper_step_angle(1140,0);


}

GetPosition(uint8_t* output)
{
  output[RXBUFFERSIZE-1] = 15;
  output[RXBUFFERSIZE-2] = position_state;
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
  HAL_FLASH_Unlock();
  if( EE_Init() != EE_OK)
  {
    uint8_t data[] = {9};
    Send_Bluettoh_Data(&data,sizeof(data));
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,UART1_rxBuffer,RXBUFFERSIZE);
  my_message_queue_init(&queue);

  LoadConfigDispenserTime();
  
  goHome();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  struct message msg;
	  enum dequeue_result result2 = my_message_queue_dequeue(&queue, &msg);
	  if (result2 == DEQUEUE_RESULT_SUCCESS) {
		  memcpy(bluetooth_rxBuffer, msg.data, RXBUFFERSIZE * sizeof(uint8_t));
		  Command();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RTC_AlarmTypeDef sAlarm = {0};

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
  sTime.Hours = 0x23;
  sTime.Minutes = 0x59;
  sTime.Seconds = 0x50;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x23;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x1;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 692;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 254;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor1_Pin|Motor2_Pin|Motor3_Pin|Motor4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOME_Pin P1_Pin P2_Pin */
  GPIO_InitStruct.Pin = HOME_Pin|P1_Pin|P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_Pin Motor2_Pin Motor3_Pin Motor4_Pin */
  GPIO_InitStruct.Pin = Motor1_Pin|Motor2_Pin|Motor3_Pin|Motor4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : P3_Pin */
  GPIO_InitStruct.Pin = P3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(P3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_PILULA_Pin */
  GPIO_InitStruct.Pin = SENSOR_PILULA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_PILULA_GPIO_Port, &GPIO_InitStruct);

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

