/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include "EEPROM.h"
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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;

char time[10];
char date[10];

uint8_t hours=0;
uint8_t minutes=0;
uint8_t seconds=0;
uint8_t weekDay=0;
uint8_t month=0;
uint8_t date1=0;
uint8_t year=0;

uint16_t digitalStates[80];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define DEV_ADDR 0xa0

int dataw3 = 17;
int datar3;


uint16_t message[62];
uint16_t analog[24];

enum alarm_state
{
	resetAlarm,
	notResetAlarm
};

int j=0,m=0,u=0,p=0;


////////////////////mux u saydir/////////////////////////////////////////////////////////////////////////////////
void mux(int sel){

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, (0b0001&(sel>>0)));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (0b0001&(sel>>1)));
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (0b0001&(sel>>2)));
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (0b0001&(sel>>3)));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////// muxdaki adc leri oxu///////////////////////////////////////////////////////
/*
 * muxdaki adc ve dig leri oxu
 */
void check_channels(int sel)
{
	int sampleRate=100;

		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 1000);
		float sum=0,average=0;
		for(int t=0;t<sampleRate;t++)
		{
			sum =sum + HAL_ADC_GetValue(&hadc3);
		}
		average = sum/sampleRate;
		message[j] = average;
		sum=0;
		average=0;
		HAL_ADC_Stop(&hadc3);
//
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 1000);
		for(int t=0;t<sampleRate;t++)
		{
			sum =sum + HAL_ADC_GetValue(&hadc2);
		}
		average = sum/sampleRate;
		message[j+16] = average;
		sum=0;
		average=0;
		HAL_ADC_Stop(&hadc2);

		int tam=0,qaliq=0;

		tam 	= j/4;
		qaliq 	= j%4;

		digitalStates[48+(3+tam*4-qaliq)] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)); // Dig5.1-5.16 Dig37-52
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////id deyerleri////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t id[25] = {0x200,0x201,0x202,0x203,0x204,0x205,0x206,0x207,0x208,0x209,0x20A,0x20B,0x20C,0x20D,0x20E,0x210,0x211,0x250,0x251,0x252,0x253,0x254,0x255,0x256,0x257};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CAN_TxHeaderTypeDef TxHeader[25];

CAN_RxHeaderTypeDef RxHeader;

int datacheck = 0;
uint8_t pk1=0;


uint16_t TxData[25][4];

uint8_t RxData[8];

uint32_t TxMailbox;


/*******	config  ***********/
uint16_t contactState[71] = 		{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};			//normal open is 0 , normal close 1
//uint16_t contactState[71] = 		{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};			//normal open is 0 , normal close 1
//uint16_t contactState[71] = 		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			//normal open is 0 , normal close 1
uint16_t delaySeconds[71] = 		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			//signal cixdiqdan sonra neqeder gozleyecek
uint16_t digitalInputId[71] =  		{2065,2066,2067,2068,2069,2070,2071,2072,2073,2074,2075,2076,2077,2078,2079,2080,2081,2082,2083,2084,2085,2086,2087,2088,2089,2090,2091,2092,2093,2094,2095,2096,2097,2098,2099,2100,2101,2102,2103,2104,2105,2106,2107,2108,2109,2110,2111,2112,2037,2038,2039,2040,2041,2042,2043,2044,2045,2046,2047,2048,2049,2050,2051,2052,2053,2054,2055,2056,2057,2058,2059};				//signal id leri
//uint16_t digitalInputId[77] =  		{2001,2002,2003,2004,2005,2006,2007,2008,2009,2010,2011,2012,2013,2014,2015,2016,2017,2018,2019,2020,2021,2022,2023,2024,2025,2026,2027,2028,2029,2030,2031,2032,2033,2034,2035,2036,2037,2038,2039,2040,2041,2042,2043,2044,2045,2046,2047,2048,2049,2050,2051,2052,2053,2054,2055,2056,2057,2058,2059,2060,2061,2062,2063,2064,2065,2066,2067,2068,2069,2070,2071,2072,2073,2074,2075,2076,2077};				//signal id leri
uint8_t fadeOut[71] =  			{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t fadeOutBaxmaq[71] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //hansi giris fade outdu onu tutur
uint16_t fadeOutTot[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t fadeOutTotRead[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t fadeOutTotReadTest[12] =  	{0,0,0,0,0,0,0,0,0,0,0,0}; //
fadeOutReg = 0;

uint16_t delaySecondsCount[71] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			//delay saniye sayici

uint16_t alarmOn[71] =  			{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint16_t alarmOnAnalog[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint16_t alarmCount[71] =  			{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 			// digital alarm sayicisi
uint16_t waitingForDelay[71] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			// delay ucun gozleme registeri


uint16_t analogInputID[24] =  		{2006,2008,2010,2012,2014,2016,2018,2020,2021,2022,2023,2024,2025,2026,2027,2028,2029,2030,2031,2032,2033,2034,2035,2036}; // analog ID

uint16_t secondByte[24] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog alarm sayicilarin tutmaq ucun

uint16_t analogAlarmCount[24] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog alarm sayicilarin tutmaq ucun
uint16_t analogAlarmCountDown[24] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog alarm sayicilarin tutmaq ucun

uint8_t analogFadeOut[24] =  		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog fade outlari tutmaq ucun
uint8_t analogFadeOutBaxmaq[24] =  		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog fade outlari tutmaq ucun
uint16_t analogFadeOutTot[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t analogFadeOutTotRead[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t analogFadeOutTotReadTest[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; //
uint16_t analogSignalFoult[24] =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // analog signal foult tutucu

uint16_t stationAlarm=0;

uint8_t recivedReset=0;

//uint16_t digital[16]=0;

uint16_t digitalSum[10] = {0,0,0,0,0,0,0,0,0,0};

uint16_t intPart[50];
uint8_t fractionPart[50];

uint16_t secondWord[50];

float hValue=0;   // grafiki cixan hesabin rahat hesablamaq ucun
float hValue2=0; //ustu 2
float hValue3=0; //ustu 3
float hValueM=0;   // vurulmasi
float hValue2M=0; //2 vurulmasi
float hValue3M=0; //3 vurulmasi

float realVal[50];
float voltValIncorrect[50];
float voltVal[50];

int recivedID=0;

int say=0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{

	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
		if(RxHeader.StdId == 0x203)
		{
			pk1 = RxData[0];
			datacheck = 1;

		}

		if(RxHeader.StdId == 0x500)
		{
			recivedReset = 1;
			//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
		}

		if(RxHeader.StdId == 0x600)
		{
			fadeOutReg = 1;

			recivedID = (int)(RxData[0]) + ((int)(RxData[1]) << 8);

			for(int t=0;t<71;t++)
			{
				if(digitalInputId[t] == recivedID)
				{
					fadeOut[t] = RxData[2];

				}

				if(t < 24)
				{
					if(analogInputID[t] == recivedID)
					{
						analogFadeOut[t] = RxData[2];
					}
				}

			}


		}

		if(RxHeader.StdId == 0x501)
		{
			pk1 = RxData[0];
		}

		if(RxHeader.DLC == 2)
		{

		}

}


struct analogConfig
{
	float minVolt;
	float maxVolt;
	float minRealVal;
	float maxRealVal;
	float warningLevel;
	float alarmLevel;
	int moreThen;
	//hecne baglanmayanlara 1500 limit qoy ve boyukdur deki yalanci alarm yaratmasin

}analogConfigs[25]={
		{0.64,3.3,0,114.296875,40,70,1},		//2006
		{0.64,3.3,0,114.296875,60,98,1},		//2008
		{0.64,3.3,0,114.296875,60,80,1},		//2010
		{0.64,3.3,0,114.296875,60,65,1},		//2012
		{0.64,3.3,0,114.296875,60,90,1},		//2014
		{0.64,3.3,0,114.296875,40,85,1},		//2016
		{0.64,3.3,0,114.296875,60,1500,1},	//2018//bosdur
		{0.64,3.3,0,114.296875,60,1500,1},	//2020//bosdur
		{0.64,3.2,0,4,60,0.5,0},			//2021
		{0.64,3.2,0,10,60,2,0},				//2022
		{0.64,3.2,0,50,40,1500,1},			//2023//bosdur
		{0.64,3.2,0,10,60,1500,1},			//2024//bosdur
		{0.64,3.2,0,1000,1000,1300,1},		//2025
		{0.64,3.2,0,6,60,0.7,0},			//2026
		{0.64,3.2,0,4,60,0.3,0},			//2027
		{0.64,3.2,0,1.5,40,0.25,0},			//2028
		{0.64,3.2,0,15,60,8,1},				//2029
		{0.64,3.2,0,6,60,0.7,0},			//2030
		{0.64,3.2,0,10,60,3,0},				//2031
		{0.64,3.2,0,4,60,0.6,0},			//2032
		{0.64,3.2,0,10,40,1,0},				//2033
		{0.64,3.2,0,600,60,490,1},			//2034
		{0.64,3.2,0,100,60,1500,1},			//2035
		{0.64,3.2,0,6,60,1500,1},			//2036
		{0.64,3.2,0,1000,60,70}
};
/*
struct analogConfig
{
	float minVolt;
	float maxVolt;
	float minRealVal;
	float maxRealVal;
	float warningLevel;
	float alarmLevel;
	int moreThen;
	//hecne baglanmayanlara 1500 limit qoy ve boyukdur deki yalanci alarm yaratmasin
}analogConfigs[25]={
		{0.64,3.3,0,103.90625,40,1500,1},		//2006
		{0.64,3.3,0,103.90625,60,1500,1},	//2008//bosdur
		{0.64,3.3,0,103.90625,60,1500,1},		//2010
		{0.64,3.3,0,103.90625,60,1500,1},		//2012
		{0.64,3.3,0,103.90625,60,1500,1},	//2014
		{0.64,3.3,0,103.90625,40,1500,1},		//2016
		{0.64,3.3,0,103.90625,60,1500,1},	//2018//bosdur
		{0.64,3.3,0,103.90625,60,1500,1},	//2020//bosdur
		{0.64,3.2,0,4,60,1500,1},			//2021
		{0.64,3.2,0,10,60,1500,1},				//2022
		{0.64,3.2,0,50,40,1500,1},			//2023//bosdur
		{0.64,3.2,0,10,60,1500,1},			//2024//bosdur
		{0.64,3.2,0,1000,1000,1500,1},		//2025
		{0.64,3.2,0,6,60,1500,1},			//2026
		{0.64,3.2,0,4,60,1500,1},			//2027
		{0.64,3.2,0,1.5,40,1500,1},			//2028
		{0.64,3.2,0,15,60,1500,1},				//2029
		{0.64,3.2,0,6,60,1500,1},			//2030
		{0.64,3.2,0,10,60,1500,1},				//2031
		{0.64,3.2,0,4,60,1500,1},				//2032
		{0.64,3.2,0,10,40,1500,1},			//2033
		{0.64,3.2,0,600,60,1500,1},			//2034
		{0.64,3.2,0,100,60,1500,1},			//2035
		{0.64,3.2,0,6,60,1500,1},			//2036
		{0.64,3.2,0,1000,1500,1}
};
*/

void set_time (void)
{
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

	  sTime.Hours = 0x14;
	  sTime.Minutes = 0x14;
	  sTime.Seconds = 0x30;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
	  sDate.Month = RTC_MONTH_JULY;
	  sDate.Date = 0x5;
	  sDate.Year = 0x22;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }



	  /* USER CODE BEGIN RTC_Init 2 */


	  /* USER CODE END RTC_Init 2 */
}


void get_time(void)
{
// Get the RTC current Time
 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
// Get the RTC current Date
 HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
// Display time Format: hh:mm:ss
 sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);
// Display date Format: dd-mm-yy
 sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
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
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
// 14 dene header olacaq

  for(j=0;j<25;j++){
	  TxHeader[j].DLC = 8;
	  TxHeader[j].IDE = CAN_ID_STD;
	  TxHeader[j].RTR = CAN_RTR_DATA;
	  TxHeader[j].StdId = id[j];
  }

  //Start Timer

  HAL_TIM_Base_Start_IT(&htim6);

  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
  {
	  set_time();
  }

  /*
  EEPROM_PageErase(0);
  EEPROM_PageErase(1);
  EEPROM_PageErase(2);
  EEPROM_PageErase(3);
  EEPROM_PageErase(4);
  EEPROM_PageErase(5);
  EEPROM_PageErase(6);
  EEPROM_PageErase(7);
*/


/**/
  fadeOutTotRead[0] = EEPROM_Read_NUM(1, 0);
  fadeOutTotRead[1] = EEPROM_Read_NUM(2, 0);
  fadeOutTotRead[2] = EEPROM_Read_NUM(3, 0);
  fadeOutTotRead[3] = EEPROM_Read_NUM(4, 0);
  fadeOutTotRead[4] = EEPROM_Read_NUM(5, 0);

  analogFadeOutTotRead[0] = EEPROM_Read_NUM(6, 0);
  analogFadeOutTotRead[1] = EEPROM_Read_NUM(7, 0);

  for(int k=0;k<16;k++)
  {
	  fadeOut[k] = (fadeOutTotRead[0] >> k) & 1;
	  fadeOut[k+16] = (fadeOutTotRead[1] >> k) & 1;
	  fadeOut[k+32] = (fadeOutTotRead[2] >> k) & 1;
	  fadeOut[k+48] = (fadeOutTotRead[3] >> k) & 1;
	  if(k<7)
	  {
		  fadeOut[k+64] = (fadeOutTotRead[4] >> k) & 1;
	  }

	  analogFadeOut[k] = (analogFadeOutTotRead[0] >> k) & 1;

	  if(k<8)
	  {
		  analogFadeOut[k+16] = (analogFadeOutTotRead[1] >> k) & 1;
	  }

  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(fadeOutReg == 1)
	  {
		  fadeOutTot[0] = 0;
		  fadeOutTot[1] = 0;
		  fadeOutTot[2] = 0;
		  fadeOutTot[3] = 0;
		  fadeOutTot[4] = 0;
		  analogFadeOutTot[0] = 0;
		  analogFadeOutTot[1] = 0;

		  for(int t=0;t<16;t++)
		  {
			  fadeOutTot[0] |= fadeOut[t]<<t;
			  fadeOutTot[1] |= fadeOut[t+16]<<t;
			  fadeOutTot[2] |= fadeOut[t+32]<<t;
			  fadeOutTot[3] |= fadeOut[t+48]<<t;
			  if(t<7)
			  {
				  fadeOutTot[4] |= fadeOut[t+64]<<t;
			  }

			  analogFadeOutTot[0] |= analogFadeOut[t]<<t;

			  if(t<8)
			  {
				  analogFadeOutTot[1] |= analogFadeOut[t+16]<<t;
			  }

		  }


		  EEPROM_Write_NUM (1, 0, fadeOutTot[0]);
		  EEPROM_Write_NUM (2, 0, fadeOutTot[1]);
		  EEPROM_Write_NUM (3, 0, fadeOutTot[2]);
		  EEPROM_Write_NUM (4, 0, fadeOutTot[3]);
		  EEPROM_Write_NUM (5, 0, fadeOutTot[4]);

		  EEPROM_Write_NUM (6, 0, analogFadeOutTot[0]);
		  EEPROM_Write_NUM (7, 0, analogFadeOutTot[1]);

		  fadeOutReg = 0;
	  }

	  fadeOutTotReadTest[0] = EEPROM_Read_NUM(1, 0);
	  fadeOutTotReadTest[1] = EEPROM_Read_NUM(2, 0);
	  fadeOutTotReadTest[2] = EEPROM_Read_NUM(3, 0);
	  fadeOutTotReadTest[3] = EEPROM_Read_NUM(4, 0);
	  fadeOutTotReadTest[4] = EEPROM_Read_NUM(5, 0);

	  analogFadeOutTotReadTest[0] = EEPROM_Read_NUM(6, 0);
	  analogFadeOutTotReadTest[1] = EEPROM_Read_NUM(7, 0);

	  for(int k=0;k<16;k++)
	  {
		  fadeOutBaxmaq[k] = (fadeOutTotReadTest[0] >> k) & 1;
		  fadeOutBaxmaq[k+16] = (fadeOutTotReadTest[1] >> k) & 1;
		  fadeOutBaxmaq[k+32] = (fadeOutTotReadTest[2] >> k) & 1;
		  fadeOutBaxmaq[k+48] = (fadeOutTotReadTest[3] >> k) & 1;
		  if(k<7)
		  {
			  fadeOutBaxmaq[k+64] = (fadeOutTotReadTest[4] >> k) & 1;
		  }

		  analogFadeOutBaxmaq[k] = (analogFadeOutTotReadTest[0] >> k) & 1;

		  if(k<8)
		  {
			  analogFadeOutBaxmaq[k+16] = (analogFadeOutTotReadTest[1] >> k) & 1;
		  }

	  }


	  get_time();

	  hours = gTime.Hours;
	  minutes= gTime.Minutes;
	  seconds =gTime.Seconds;

	  ////////////////////////////////////////////////mux u saydir adc ve dig deyerlri yolla/////////////////////////////////////////////////////
	  	  for (j = 0; j < 16; ++j) {
	  	  		mux(15-j);
	  	  		HAL_Delay(1);
	  	  		check_channels(j);
	  	  	}

	  	  for(j=0;j<32;j++)
	  	  {
	  		  if((j != 0) && (j != 2) && (j != 4) && (j != 6) && (j != 8) && (j != 10) && (j != 12) && (j != 14))
	  		  {
	  			  analog[m]=message[j]; ////lazimsiz bos mesajlari atmaq
	  			  m++;
	  		  }
	  	  }
	  	  m=0;
	  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  	/////////////////////////////////////////////tek oxunan digitallar//////////////////////////////////////////////////////////////////////
	  		  	  /*
	  		  	   * digital deyerleri oxu ve surusturerek 1 wordluk sum a yaz
	  		  	   */

	  		  	  ///////////////////////////////////16 digital/////////////////////////////////  ///digitallari arraya duzmek
	  		  	  digitalStates[67] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);	//dig 1.1	//dig 53
	  		  	  digitalStates[66] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);	//dig 1.2	//dig 54
	  		  	  digitalStates[65] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);	//dig 1.3	//dig 55
	  		  	  digitalStates[64] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);	//dig 1.4	//dig 56
	  		  	  digitalStates[68] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);	//dig 1.5	//dig 57
	  		  	  digitalStates[69] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15);	//dig 1.6	//dig 58
	  		  	  digitalStates[70] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);	//dig 1.7	//dig 59
	  		  	//digitalStates[71] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);	//dig 1.8	//dig 60
	  		  	//digitalStates[72] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);	//dig 1.9	//dig 61
	  		  	//digitalStates[73] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);	//dig 1.10	//dig 62
	  		  	//digitalStates[74] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);	//dig 1.11	//dig 63
	  		  	//digitalStates[75] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);	//dig 1.12	//dig 64
	  		  	//digitalStates[76]	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);	//dig 1.13	//dig 1
	  		  	//digitalStates[77] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);	//dig 1.14	//dig 2
	  		  	//digitalStates[78] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);	//dig 1.15	//dig 3
	  		  	//digitalStates[79]	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);	//dig 1.16	//dig 4



	  		  	  ///////////////////////////////////16 digital/////////////////////////////////  ///digitallari arraya duzmek
	  		  	  digitalStates[0]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3);	//dig 2.1	//dig 65
	  			  digitalStates[1]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);	//dig 2.2	//dig 66
	  			  digitalStates[2]	= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);	//dig 2.3	//dig 67
	  			  digitalStates[3]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4);	//dig 2.4	//dig 68
	  			  digitalStates[4]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13);	//dig 2.5	//dig 69
	  			  digitalStates[5]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12);	//dig 2.6	//dig 70
	  			  digitalStates[6]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);	//dig 2.7	//dig 71
	  			  digitalStates[7]	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14);	//dig 2.8	//dig 72
	  			  digitalStates[8]	= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2);	//dig 2.9	//dig 73
	  			  digitalStates[9]	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);	//dig 2.10	//dig 74
	  			  digitalStates[10]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);	//dig 2.11	//dig 75
	  			  digitalStates[11]	= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);	//dig 2.12	//dig 76
	  			  digitalStates[12]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);	//dig 2.13	//dig 77
	  			  digitalStates[13]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);	//dig 2.14	//dig 78
	  			  digitalStates[14]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);	//dig 2.15	//dig 79
	  			  digitalStates[15]	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);	//dig 2.16	//dig 80

	  			  //////////////////////////////////16 digital//////////////////////////////////  ///digitallari arraya duzmek
	  			  digitalStates[16] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);	//dig 3.1	//dig 81
	  			  digitalStates[17] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);	//dig 3.2	//dig 82
	  			  digitalStates[18] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);	//dig 3.3	//dig 83
	  			  digitalStates[19] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);	//dig 3.4	//dig 84
	  			  digitalStates[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);	//dig 3.5	//dig 85
	  			  digitalStates[21] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);	//dig 3.6	//dig 86
	  			  digitalStates[22] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);	//dig 3.7	//dig 87
	  			  digitalStates[23] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);	//dig 3.8	//dig 88
	  			  digitalStates[24] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9);	//dig 3.9	//dig 89
	  			  digitalStates[25] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10);	//dig 3.10	//dig 90
	  			  digitalStates[26] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11);	//dig 3.11	//dig 91
	  			  digitalStates[27] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15);	//dig 3.12	//dig 92
	  			  digitalStates[28] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);	//dig 3.13	//dig 93
	  			  digitalStates[29] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);	//dig 3.14	//dig 94
	  			  digitalStates[30] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);	//dig 3.15	//dig 95
	  			  digitalStates[31] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);	//dig 3.16	//dig 96

	  			  //////////////////////////////////16 digital//////////////////////////////////  ///digitallari arraya duzmek
	  			  digitalStates[32] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);	//dig 4.1	//dig 97
	  			  digitalStates[33] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);	//dig 4.2	//dig 98
	  			  digitalStates[34] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);	//dig 4.3	//dig 99
	  			  digitalStates[35] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5);	//dig 4.4	//dig 100
	  			  digitalStates[36] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7); 	//dig 4.5	//dig 101
	  			  digitalStates[37] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); 	//dig 4.6	//dig 102
	  			  digitalStates[38] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8); 	//dig 4.7	//dig 103
	  			  digitalStates[39] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7); 	//dig 4.8	//dig 104
	  			  digitalStates[40] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);	//dig 4.9	//dig 105
	  			  digitalStates[41] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);	//dig 4.10	//dig 106
	  			  digitalStates[42] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);	//dig 4.11	//dig 107
	  			  digitalStates[43] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);	//dig 4.12	//dig 108
	  			  digitalStates[44] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);	//dig 4.13	//dig 109
	  			  digitalStates[45] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);	//dig 4.14	//dig 110
	  			  digitalStates[46] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);	//dig 4.15	//dig 111
	  			  digitalStates[47] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);	//dig 4.16	//dig 112
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		  		  	for(int k=0;k<71;k++)
		  		  	{
  		  		  		if((k != 61) && (k != 70))
  		  		  		{
		  		  		if(digitalStates[k] == contactState[k])   							//eger digital girisimiz biizm mueyyen elediyimiz veziyetdedise yeni loru dile yanibsa
		  		  		{
		  		  			if (alarmOn[k] == 0)    										//eger alarim cixmayibsa
		  		  			{
		  		  				if (waitingForDelay[k] == 1)    							//qoyulan vaxdin tamamlanmagin gozdeyirik
		  		  				{
		  		  					if ((delaySeconds[k] <= delaySecondsCount[k]) && (fadeOut[k] == 0))   			// qoyulan vaxda catdisa
		  		  					{
		  								alarmOn[k] = 1;                         			//alarmi yandir
		  								sendData(digitalInputId[k]);						//
		  								stationAlarm = notResetAlarm;						//alarimi yandir signal cixdi deye
		  								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);//alarim isigin yandir
		  								waitingForDelay[k] = 0;								//delay ucun gozdeme sayicisin sifirla
		  							}
		  						}
		  		  				else
		  		  				{
		  			  				alarmCount[k]++;											//n defe alarm cixidigin yoxlayan sayici
		  			  				if (alarmCount[k] >4)										//4 defe cixdisa gir
		  			  				{
		  			  					if ((delaySeconds[k] == 0) && (fadeOut[k] == 0))								//saniye sayan 0 disa gir
		  			  					{
		  									alarmOn[k] = 1;										//alari yandir
		  									sendData(digitalInputId[k]);
		  									stationAlarm = notResetAlarm;						//alarimi yandir signal cixdi deye
		  									HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);//alarim isigin yandir
		  								}
		  			  					else
		  			  					{
		  									waitingForDelay[k] = 1;								//sayci ucun gozdeme regisitiri

		  								}

		  			  					alarmCount[k] = 5;										//alarm count 4 den boyukduse 5 den cox boyumesin
		  							}
		  		  				}
		  						//delay;
		  					}
		  		  		}
		  		  		else
		  		  		{ 	//sifirla eger signal gelmeyibse hec
		  					alarmOn[k] = 0;
		  					alarmCount[k] = 0;
		  					waitingForDelay[k] = 0;
		  					delaySecondsCount[k] = 0;
		  		  		}

	  		  			u=k/8;	        // 1 id 4 word yollayir onagore her word ucun boluruk
	  		  			p=k%8;			//sonraki wordun necencisi olduguna bundan baxiriq


		  		  		if (alarmOn[k] != 0)
		  		  		{
		  		  			digitalSum[u] |= (1 << p*2); //burdada necncei wordun olduguna sonra hemin wordun necencisi olduguna baxiriq
		  		  		}
		  		  	digitalSum[u] |= (fadeOut[k] << (p*2+1));
		  		  	}
  }
		  		  	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 1 || recivedReset==1)  //reset basdiq veya conpuyuterden reset geldi
		  		  		{
		  			  		stationAlarm = resetAlarm;									//alarmi reset et
		  			  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);		//ve sondur alarmi
		  			  		recivedReset = 0;											//compyuterden gelen reseti sifirla
  		  			  		say = 10;
		  		  		}


  		  		  	if(say != 0)
  		  		  	{
  		  		  	TxData[17][1] = 3;
  		  		  	say--;
  		  		  	for(int hh=0;hh<10;hh++)
  		  		  	{
		  		  		HAL_CAN_AddTxMessage(&hcan1, &TxHeader[17], TxData[17], &TxMailbox);
		  		  		HAL_Delay(20);
  		  		  	}
  		  		  	}
  		  		  	else
  		  		  	{
  		  		  	TxData[17][1] = 0;
  		  		  	}


		  		  	TxData[17][0] = stationAlarm;
		  		  	TxData[17][1] = stationAlarm;
		  		  	TxData[17][2] = stationAlarm;
		  		  	TxData[17][3] = stationAlarm;

	  		  		HAL_CAN_AddTxMessage(&hcan1, &TxHeader[17], TxData[17], &TxMailbox);
	  		  		HAL_Delay(20);

		  		  	TxData[13][0] = digitalSum[0];
		  		  	TxData[13][1] = digitalSum[1];
		  		  	TxData[13][2] = digitalSum[2];
		  		  	TxData[13][3] = digitalSum[3];
		  		  	TxData[14][0] = digitalSum[4];
		  		  	TxData[14][1] = digitalSum[5];
		  		  	TxData[14][2] = digitalSum[6];
		  		  	TxData[14][3] = digitalSum[7];
		  		  	TxData[15][0] = digitalSum[8];
		  		  	TxData[15][1] = digitalSum[9];
		  		  	TxData[15][2] = 0;
		  		  	TxData[15][3] = 0;

		  		  	HAL_CAN_AddTxMessage(&hcan1, &TxHeader[13], TxData[13], &TxMailbox);
		  		  	HAL_Delay(20);
		  			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[14], TxData[14], &TxMailbox);
		  			HAL_Delay(20);
		  			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[15], TxData[15], &TxMailbox);
		  			HAL_Delay(20);

		  			digitalSum[0] = 0;
		  			digitalSum[1] = 0;
		  			digitalSum[2] = 0;
		  			digitalSum[3] = 0;
		  			digitalSum[4] = 0;
		  			digitalSum[5] = 0;
		  			digitalSum[6] = 0;
		  			digitalSum[7] = 0;
		  			digitalSum[8] = 0;
		  			digitalSum[9] = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	for(int i=0;i<12;i++)
			  {
				  for (j=0;j<2;j++)
				  {
					  /*
					  voltValIncorrect[i*2+j] = (((float)analog[i*2+j])*3.3)/4096.0;    //voltaj deyeri xetali olan
					  if(voltValIncorrect[i*2+j] <= 1.6)
					  {
						voltVal[i*2+j] = voltValIncorrect[i*2+j];    //voltaj deyeri tap
					  }
					  else
					  {
						  hValue  = (voltValIncorrect[i*2+j] - 1.6);
						  hValue2 = pow(hValue,2);
						  hValue3 = pow(hValue,3);
  						  hValueM  = 0.1990*hValue;
  						  hValue2M = 0.4346*hValue2;
  						  hValue3M = 0.44*hValue3;

						voltVal[i*2+j] = voltValIncorrect[i*2+j] + hValue3M - hValue2M + hValueM - 0,0072;
					  }
					  */
					  voltVal[i*2+j] = (((float)analog[i*2+j])*3.3)/4096.0;
					  realVal[i*2+j] = ((voltVal[i*2+j] - analogConfigs[i*2+j].minVolt )/(analogConfigs[i*2+j].maxVolt-analogConfigs[i*2+j].minVolt))*(analogConfigs[i*2+j].maxRealVal - analogConfigs[i*2+j].minRealVal);   //olculen vahide gore hesablanan deyer yeni tempdise tempratur qarsiligi voltajin
					  intPart[i*2+j] = (uint16_t)realVal[i*2+j];
					  fractionPart[i*2+j] = (uint8_t)((realVal[i*2+j] - intPart[i*2+j])*100);

  					  if(analogConfigs[i*2+j].moreThen == 0)
  					  {
					  if(realVal[i*2+j] < analogConfigs[i*2+j].alarmLevel)
					  {
						  analogAlarmCountDown[i*2+j] = 0;
							if(alarmOnAnalog[i*2+j] == 0)
							{
	  							analogAlarmCount[i*2+j]++; // analog alarimin say
						  if((analogAlarmCount >= 10) && (analogFadeOut[i*2+j] == 0)) // 4 defe alarm verse analog alarimin yandir
						  {
							alarmOnAnalog[i*2+j] = 1;
							sendData(analogInputID[i * 2 + j]);
							secondByte[i*2+j] |= 2;    // eger alarim oldusa 1 ci biti 1 ele
							stationAlarm = notResetAlarm;		//alarm cixdi
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //alarm isigin yandir
							analogAlarmCount[i*2+j] = 10;   //analog sayicisi 4 e catdisa 4 de saxla
						  }
							}
					  }
					  else
					  {
							analogAlarmCountDown[i*2+j]++;
							if(analogAlarmCountDown[i*2+j] >= 10)
							{
								analogAlarmCount[i*2+j] = 0;  //alarim deyilse sayicini sifirla
								alarmOnAnalog[i*2+j] = 0;
								analogAlarmCountDown[i*2+j] = 10;
								secondByte[i*2+j] &= ~2;
							}
					  }
  					  }
  					  else if(analogConfigs[i*2+j].moreThen == 1)
  					  {
  						  if(realVal[i*2+j] > analogConfigs[i*2+j].alarmLevel)
  						  {
  							  analogAlarmCountDown[i*2+j] = 0;
	  							if(alarmOnAnalog[i*2+j] == 0)
	  							{
	  								analogAlarmCount[i*2+j]++; // analog alarimin say
  							  if((analogAlarmCount >= 10) && (analogFadeOut[i*2+j] == 0)) // 4 defe alarm verse analog alarimin yandir
  							  {
  								secondByte[i*2+j] |= 2;    // eger alarim oldusa 1 ci biti 1 ele
  								alarmOnAnalog[i*2+j] = 1;
  								sendData(analogInputID[i * 2 + j]);
  								stationAlarm = notResetAlarm;		//alarm cixdi
  								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //alarm isigin yandir
  								analogAlarmCount[i*2+j] = 10;   //analog sayicisi 4 e catdisa 4 de saxla
  							  }
	  							}
  						  }
  						  else
  						  {
  							analogAlarmCountDown[i*2+j]++;
  							if(analogAlarmCountDown[i*2+j] >= 10)
  							{
  								alarmOnAnalog[i*2+j] = 0;
  								analogAlarmCountDown[i*2+j] = 10;
  								analogAlarmCount[i*2+j] = 0;  //alarim deyilse sayicini sifirla
  								secondByte[i*2+j] &= ~2;
  							}
  						  }
  					  }

  					  /*
					  if(realVal[i*2+j] > analogConfigs[i*2+j].warningLevel)   //worning levele geldise
					  {
						  secondByte |=1;  // 0 ci biti 1 ele
					  }
  					   */


					  if(analogFadeOut[i*2+j] == 1)
					  {
						  secondByte[i*2+j] |=4;
					  }
  					  else
  					  {
  						secondByte[i*2+j] &= ~4;
  					  }

					  if(analogSignalFoult[i*2+j] == 1)
					  {
						  secondByte[i*2+j] |=8;
					  }
  					  else
  					  {
  						secondByte[i*2+j] &= ~8;
  					  }

					  secondWord[i*2+j] = (uint16_t)secondByte[i*2+j] + ((uint16_t)fractionPart[i*2+j])*256;

					  TxData[i][j*2] = intPart[i*2+j];
					  TxData[i][j*2+1] = secondWord[i*2+j];

				  }

				  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[i], TxData[i], &TxMailbox);
				  HAL_Delay(20);

			  }

			  TxData[16][0] = digitalSum;
			  TxData[16][1] = digitalSum;
			  TxData[16][2] = digitalSum;
			  TxData[16][3] = digitalSum;

			  HAL_CAN_AddTxMessage(&hcan1, &TxHeader[16], TxData[16], &TxMailbox);
			  HAL_Delay(20);



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 64;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

        canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
        canfilterconfig.FilterBank = 10;
        canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        canfilterconfig.FilterIdHigh = 0; //0x446<<5;
        canfilterconfig.FilterIdLow = 0;
        canfilterconfig.FilterMaskIdHigh = 0; //0x446<<5;
        canfilterconfig.FilterMaskIdLow = 0;
        canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
        canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
        canfilterconfig.SlaveStartFilterBank = 0;

        HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  sTime.Hours = 0x14;
  sTime.Minutes = 0x14;
  sTime.Seconds = 0x30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x5;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */


  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM30_GPIO_Port, ALARM30_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SEL3_Pin|SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SEL1_Pin|SEL0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D2_9_Pin D2_10_Pin D1_11_Pin D2_12_Pin
                           D2_7_Pin D2_3_Pin D2_16_Pin */
  GPIO_InitStruct.Pin = D2_9_Pin|D2_10_Pin|D1_11_Pin|D2_12_Pin
                          |D2_7_Pin|D2_3_Pin|D2_16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_6_Pin D4_1_Pin D4_2_Pin D4_8_Pin
                           D4_7_Pin D3_10_Pin D3_11_Pin D3_12_Pin */
  GPIO_InitStruct.Pin = D2_6_Pin|D4_1_Pin|D4_2_Pin|D4_8_Pin
                          |D4_7_Pin|D3_10_Pin|D3_11_Pin|D3_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_5_Pin PF6 D1_5_Pin D1_6_Pin */
  GPIO_InitStruct.Pin = D2_5_Pin|GPIO_PIN_6|D1_5_Pin|D1_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_3_Pin D1_2_Pin D5__Pin D1_1_Pin
                           D3_9_Pin D4_5_Pin D4_6_Pin D3_13_Pin
                           D3_14_Pin D2_8_Pin */
  GPIO_InitStruct.Pin = D1_3_Pin|D1_2_Pin|D5__Pin|D1_1_Pin
                          |D3_9_Pin|D4_5_Pin|D4_6_Pin|D3_13_Pin
                          |D3_14_Pin|D2_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM30_Pin */
  GPIO_InitStruct.Pin = ALARM30_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM30_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_4_Pin D1_7_Pin PB11 D3_1_Pin
                           D3_2_Pin D3_3_Pin D3_4_Pin D2_13_Pin
                           D2_14_Pin D2_15_Pin */
  GPIO_InitStruct.Pin = D1_4_Pin|D1_7_Pin|GPIO_PIN_11|D3_1_Pin
                          |D3_2_Pin|D3_3_Pin|D3_4_Pin|D2_13_Pin
                          |D2_14_Pin|D2_15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL3_Pin SEL2_Pin */
  GPIO_InitStruct.Pin = SEL3_Pin|SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL1_Pin SEL0_Pin */
  GPIO_InitStruct.Pin = SEL1_Pin|SEL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_14_Pin D1_13_Pin D1_16_Pin D3_16_Pin
                           D3_15_Pin D4_3_Pin D4_4_Pin D3_5_Pin
                           D3_6_Pin D3_7_Pin D2_2_Pin D2_1_Pin
                           D2_4_Pin D3_8_Pin */
  GPIO_InitStruct.Pin = D1_14_Pin|D1_13_Pin|D1_16_Pin|D3_16_Pin
                          |D3_15_Pin|D4_3_Pin|D4_4_Pin|D3_5_Pin
                          |D3_6_Pin|D3_7_Pin|D2_2_Pin|D2_1_Pin
                          |D2_4_Pin|D3_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_12_Pin D4_11_Pin D4_10_Pin D4_9_Pin */
  GPIO_InitStruct.Pin = D4_12_Pin|D4_11_Pin|D4_10_Pin|D4_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
		for(int h=0;h<16;h++)
		{
			if(waitingForDelay[h] == 1)
			{
				delaySecondsCount[h]++;
				if(delaySecondsCount[h] >= 50)
				{
					//delaySecondsCount[h] = 50;
				}
			}
		}

	}
}



void sendData(int inputId)
{
	TxData[18][0]=inputId;  ////giris nomresi
	//TxData[18][1]=(uint16_t)seconds;
	TxData[18][2]=2;  /////////stansiya nomresi
	TxData[18][3]=4;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader[18], TxData[18], &TxMailbox);
	HAL_Delay(20);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
