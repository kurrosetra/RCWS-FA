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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "stm_hal_serial.h"
#include "rwsCanID.h"
#include "lib_kontrol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
	float f;
	uint32_t u32;
	int32_t i32;
	uint8_t b[4];
} Union_u;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/
#define DEBUG						1
#if DEBUG==1
#	define DEBUG_BUTTON				1
#	define DEBUG_BUS				0
#	define DEBUG_CAM				0
#	define DEBUG_PC					0
#	define DEBUG_MOVEMENT			0
#	define DEBUG_GPS				0
#endif	//if DEBUG==1
#define MOVEMENT_CHANGE_ENABLE		1
#if DEBUG==1
#	if MOVEMENT_CHANGE_ENABLE==1
#		define DEBUG_TRACK				1
#		define DEBUG_STAB				1
#		define DEBUG_MEMORY				0
#	endif	//if MOVEMENT_CHANGE_ENABLE==1
#endif	//if DEBUG==1

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define UART_BUFSIZE	RING_BUFFER_SIZE

uint16_t bufLen = 0;
char buf[UART_BUFSIZE];

Ring_Buffer_t tx1Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx1Buffer = { { 0 }, 0, 0 };
TSerial pc = { &rx1Buffer, &tx1Buffer, &huart1 };

#if DEBUG==1
Ring_Buffer_t tx2Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx2Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx2Buffer, &tx2Buffer, &huart2 };

char vt100_home[10];
#define DEBUG_LINE_MAX		25
#define DEBUG_LINE_SIZE		16
char vt100_lineX[DEBUG_LINE_MAX][DEBUG_LINE_SIZE];
#endif	//if DEBUG==1

Ring_Buffer_t tx3Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx3Buffer = { { 0 }, 0, 0 };
TSerial button = { &rx3Buffer, &tx3Buffer, &huart3 };
uint8_t buttonLed = 0;

#define BUTTON_BUFSIZE			UART_BUFSIZE
char bufButton[BUTTON_BUFSIZE];
uint32_t buttonState = 0;

typedef struct
{
	uint8_t movement;
	uint8_t camSel;
	uint8_t camZoom;
	uint8_t camFocus;
	uint8_t lrf;
	uint8_t trk;
	uint8_t trigger;
} ButtonState_t;
ButtonState_t btnState;

const float joystickMovementMaxValue = 1000.0f;
typedef struct
{
	uint8_t id;
	uint8_t dtab;
	int azimuth;	// scale -1000 to +1000
	int elevation;	// scale -1000 to +1000
} Joystick_t;
Joystick_t jRight = { 0, 0, 0, 0 };
Joystick_t jLeft = { 1, 0, 0, 0 };

typedef enum
{
	MTR_SPD_MID = 2,
	MTR_SPD_LOW = 0,
	MTR_SPD_HIGH = 3
} MotorSpeed_e;

#define MOTOR_UPDATE_TIMEOUT	10

//#define MTR_SPEED_MAX			116500		//motor max speed=1750rpm
#define MTR_SPEED_MAX_IN_C		115000

#define MTR_PAN_SPEED_MAX_C			MTR_SPEED_MAX_IN_C
#define MTR_PAN_SPEED_MAX_DEG		C_TO_DEG_AZ(MTR_PAN_SPEED_MAX_C)

#define MTR_TILT_SPEED_MAX_C		75000
#define MTR_TILT_SPEED_MAX_DEG		C_TO_DEG_AZ(MTR_TILT_SPEED_MAX_C)
const float MTR_PAN_SPEED_DEG[3] = { 0.30864032150016f, 5.58988613529489f, 70.0f };
const float MTR_TILT_SPEED_DEG[3] = { 0.30551921320896f, 4.91854108348383f, 60.0f };

float panMoveMax = 0.0f;
float tiltMoveMax = 0.0f;
uint8_t manualSpeedMaxIndex = 0;

#define motorEnable()			bitSet(canSendStabMode.data[0], 7)
#define motorDisable()			bitClear(canSendStabMode.data[0], 7)

typedef enum
{
	TRK_GATE_NONE = 0,
	TRK_GATE_BIGGER = 1,
	TRK_GATE_SMALLER = -1
} TrackGateResize_e;
int8_t trackGateResize = 0;

typedef enum
{
	TRK_SEL_NONE = 0,
	TRK_SEL_NEXT = 1,
	TRK_SEL_PREV = -1
} TrackSelectTarget_e;
int8_t trackSelectTarget = TRK_SEL_NONE;

typedef enum
{
	CAM_ST_NONE = 2,
	CAM_ST_SONY = 0,
	CAM_ST_THERMAL = 3
} CameraState_e;

#define WEAPON_FIRE_OFF()	(htim12.Instance->CCR2 = TRIGGER_OFF)
#define WEAPON_FIRE_ON()	(htim12.Instance->CCR2 = TRIGGER_ON)
typedef enum
{
	ST_FIRE_COUNT_INF = 3,
	ST_FIRE_COUNT_1 = 0,
	ST_FIRE_COUNT_3 = 1
} Fire_State_e;
uint8_t fireState = ST_FIRE_COUNT_INF;

typedef enum
{
	TIME_FIRE_COUNT_INF = 0,
	TIME_FIRE_COUNT_1 = 35,
	TIME_FIRE_COUNT_3 = 100
} Fire_Time_e;

typedef struct
{
	uint8_t counter;
	uint16_t distance;
} LRF_t;
LRF_t lrfData = { 0, 1000 };
float ypr[3] = { 0.0f, 0.0f, 0.0f };

const int xCross[4] = { 26, 0, 0, 0 };
const int yCross[4] = { 6, 0, 0, 0 };
uint8_t zoomValueFromOptCam = 0;
typedef struct
{
	uint8_t state;
	int x;
	int y;
} Crosshair_t;
Crosshair_t cross = { 0, 0, 0 };

typedef enum
{
	MOVE_TRAVEL_bit = 0,
	MOVE_STAB_bit,
	MOVE_TRACK_bit,
	MOVE_MEMORY_bit
} MovementMode_e;
uint8_t movementMode = 0;

#define TRK_BUFSIZE						128

typedef struct
{
	char buf[TRK_BUFSIZE];
	int16_t trkX;
	uint8_t stateX; /* 0 = PID const normal; 1 = PID const aggressive */
	int16_t trkY;
	uint8_t stateY; /* 0 = PID const normal; 1 = PID const aggressive */
	uint8_t id[5];
	uint8_t activeID;
} TTrackerData;
TTrackerData trackerData = { { 0 }, 0, 0, 0, 0, { 0 }, 0 };
int16_t trkXBalistikOffset = 0;
int16_t trkYBalistikOffset = 0;

typedef enum
{
	MOVEMENT_MODE_OFF,
	MOVEMENT_MODE_ON
} MOVEMENT_MODE_e;

uint8_t trackingMode = MOVEMENT_MODE_OFF;
uint8_t stabilizeMode = MOVEMENT_MODE_OFF;
volatile float motorPosActual[2] = { 0.0f, 0.0f };
volatile float motorVeloActual[2] = { 0.0f, 0.0f };

#define CAN_BUFSIZE							8

CAN_TxHeaderTypeDef can1TxHeader;
CAN_RxHeaderTypeDef can1RxHeader;
uint32_t can1TxMailBox;
uint8_t can1TxBuffer[CAN_BUFSIZE];
uint8_t can1RxBuffer[CAN_BUFSIZE];
const uint32_t CAN_RECEIVE_TIMEOUT = 500;

volatile bool canTransmitState = false;

typedef struct
{
	uint32_t id;
	volatile bool state;
	uint8_t data[8];
	uint8_t size;
	uint8_t online;
} TCanRecvBuffer;

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t size;
} TCanSendBuffer;

TCanRecvBuffer canRecvMotorState = { CAN_ID_RWS_MOTOR, false, { 0 }, 6, 0 };
TCanRecvBuffer canRecvMotorAngle = { CAN_ID_RWS_MTR_STAB_ANGLE, false, { 0 }, 8, 0 };
TCanRecvBuffer canRecvMotorSpeed = { CAN_ID_RWS_MTR_STAB_SPD, false, { 0 }, 8, 0 };
TCanRecvBuffer canRecvOptLrf = { CAN_ID_RWS_OPT_LRF, false, { 0 }, 3, 0 };
TCanRecvBuffer canRecvOptCam = { CAN_ID_RWS_OPT_CAM, false, { 0 }, 1, 0 };
TCanRecvBuffer canRecvStackerYPR = { CAN_ID_RWS_STAB_YPR, false, { 0 }, 6, 0 };

TCanSendBuffer canSendMotorCommand = { CAN_ID_RWS_PNL_MTR, { 0 }, 8 };
TCanSendBuffer canSendButton = { CAN_ID_RWS_BUTTON, { 0 }, 3 };
TCanSendBuffer canSendStabMode = { CAN_ID_RWS_PNL_STAB_MODE, { 0 }, 1 };
TCanSendBuffer canSendStabCorrection = { CAN_ID_RWS_PNL_STAB_CORR_STB, { 0 }, 8 };
TCanSendBuffer canSendTrackCorrection = { CAN_ID_RWS_PNL_STAB_CORR_TRK, { 0 }, 8 };

volatile uint32_t cRecvMtrState = 0;
volatile uint32_t cRecvMtrAngle = 0;
volatile uint32_t cRecvMtrVelo = 0;
volatile uint32_t cRecvOptLrf = 0;
volatile uint32_t cRecvOptCam = 0;
volatile uint32_t cRecvStab = 0;
volatile uint16_t cAngleVelo[2] = { 0, 0 };
volatile uint32_t cRecvStackerYpr = 0;

volatile uint32_t cSendMtrCmd = 0;
volatile uint32_t cSendButtonCmd = 0;
volatile uint32_t cSendStabCmd = 0;

uint32_t cTrackTelemetry = 0;

uint8_t battVolt = 0;

typedef enum
{
	Weapon_1270,
	Weapon_762
} WeaponType_e;
uint8_t WeaponType = Weapon_1270;

/* 12.7mm
 * 300m		-> 3
 * 500m		-> 5
 * 1000m	-> 10
 */
uint16_t GratW1270_300m[3][2] = { { 0, 10 }, { 0, 80 }, { 0, 160 } };
uint16_t GratW1270_500m[3][2] = { { 0, 15 }, { 0, 125 }, { 0, 290 } };
uint16_t GratW1270_1000m[3][2] = { { 0, 22 }, { 0, 257 }, { 0, 290 } };

float BalFromGratW1270[3][2] = { { 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 } };

/* tes grat

 $GRAT,0,300,0,0,10*
 $GRAT,0,500,0,0,15*
 $GRAT,0,1000,0,0,230*

 $GRAT,0,1000,1,0,230*

 $GRAT,0,300,2,0,160*
 $GRAT,0,500,2,0,290*
 $GRAT,0,1000,2,0,290*
 $GRAT,0,1000,2,0,230*

 */

/* 7.62mm
 * 300m		-> 4
 * 500m		-> 7
 * 1000m	-> 11
 */
uint16_t GratW762_300m[3][2] = { { 0, 4 }, { 0, 25 }, { 0, 55 } };
uint16_t GratW762_500m[3][2] = { { 0, 7 }, { 0, 44 }, { 0, 85 } };
uint16_t GratW762_1000m[3][2] = { { 0, 10 }, { 0, 65 }, { 0, 128 } };

float BalFromGratW762[3][2] = { { 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 } };

/* tes grat

 $GRAT,1,300,0,0,4*
 $GRAT,1,500,0,0,7*
 $GRAT,1,1000,0,0,10*

 $GRAT,1,300,2,0,55*
 $GRAT,1,500,2,0,85*
 $GRAT,1,1000,2,0,128*
 */

typedef enum
{
	MTR_STATE_STAB_ELEVATION_bit = 0,
	MTR_STATE_STAB_AZIMUTH_bit,
	MTR_STATE_TRACK_bit,
	MTR_STATE_MEMORY_bit
} MotorBitState;

const uint32_t MOTOR_MEMORY_TIMEOUT = 5000;
uint32_t motorMemoryTimer = 0;

#define PC_UPDATE_BUFSIZE	50
typedef struct
{
	char imuBuffer[PC_UPDATE_BUFSIZE];
	char crossBuffer[PC_UPDATE_BUFSIZE];
	char lrfBuffer[PC_UPDATE_BUFSIZE];
	char fovBuffer[PC_UPDATE_BUFSIZE];
	char graticuleBuffer[PC_UPDATE_BUFSIZE];
	char gateBuffer[PC_UPDATE_BUFSIZE];
	char moveBuffer[PC_UPDATE_BUFSIZE];
} PcStringUpdate_t;
PcStringUpdate_t pcUpdateBuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static long constrain(long x, long min, long max);
static long map(long x, long in_min, long in_max, long out_min, long out_max);
static char** str_split(char* a_str, const char a_delim);

static void CAN_Config();
static void canRecvHandler();

static uint16_t battVoltActual(uint8_t);
static void crossAdjust(int x, int y);
static void pcHandler();

static void buttonHandler();
static void buttonPanelUpdate(uint32_t millis, uint32_t btn);
static void buttonJoystickUpdate(uint32_t millis);

static void motorHandler();
//static int32_t motorJoystickConvert(int input, long _spdMax);
static float motorJoystickConvert(int input, float _spdMax);
static void motorUpdateData(uint8_t enable, float pan, float tilt);
static void motorJoystickHandler(uint32_t millis);
#if MOVEMENT_CHANGE_ENABLE==1
static void motorStabHandler(uint32_t millis);
static void motorTrackingHandler(uint32_t millis);
static void motorMemoryHandler(uint32_t millis);

static void motorJoystickStarting(uint32_t millis);
static void motorStabStarting(uint32_t millis);
static void motorTrackingStarting(uint32_t millis);
static void motorMemoryStarting(uint32_t millis);

static void trackingPidInit();
static void trackingPidDeInit();
static void stabPidInit();
static void stabPidDeInit();
#endif	//if MOVEMENT_CHANGE_ENABLE==1

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
#if DEBUG==1
	sprintf(vt100_home, "\x1b[2J\x1b[H");
	unsigned char debugLine = 0;
	for ( debugLine = 0; debugLine < DEBUG_LINE_MAX; debugLine++ )
		sprintf(vt100_lineX[debugLine], "\x1b[%d;0H\x1b[2K", debugLine);
#endif	//if DEBUG==1

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM12_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_IWDG_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Refresh(&hiwdg);

	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

#if DEBUG==1
	serial_init(&debug);

	bufLen = sprintf(buf, "%sPanel Firmware!\r\n", vt100_home);
	serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG

	serial_init(&button);
	serial_init(&pc);

	CAN_Config();

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	WEAPON_FIRE_OFF();

	/* TODO tracking init*/
	Kontrol_init();
	Kontrol_Konstanta_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis;
#if DEBUG==1
	uint32_t battTimer = 1000;

	char cmdPID[UART_BUFSIZE];
	Konstanta_PID_t pid;
#endif	//if DEBUG==1

	uint32_t _countTimer = 0;
	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		millis = HAL_GetTick();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		buttonHandler();
		canRecvHandler();
		motorHandler();
		pcHandler();

#if DEBUG==1
		if (millis >= battTimer) {
			battTimer = millis + 500;

			bufLen = sprintf(buf, "%sBattVolt= %dmV", vt100_lineX[2], battVoltActual(battVolt));
			serial_write_str(&debug, buf, bufLen);

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}

		bool cmdCompleted = false;
		if (serial_available(&debug)) {
			char c[2] = { 0, 0 };
			c[0] = serial_read(&debug);
			if (c[0] == '$')
				memset(cmdPID, 0, UART_BUFSIZE);
			else if (c[0] == '*')
				cmdCompleted = true;

			strcat(cmdPID, c);
		}

		if (cmdCompleted) {
			char *s;
			char **tokens;

			/*
			 * contoh format update pid:
			 * $PID,[ZOOM_LEVEL],Kp1,Kp2,Kd1,Kd2,Ki1,Ki2,[THRESHOLD]*
			 * $PID,0,0.0,0.0,0.0,0.0,0.0,0.0,0*
			 */

			s = strstr(cmdPID, "$PID,");
			if (s) {
				int zoomLevel;
				float f[6];
				int threshold;

				tokens = str_split(cmdPID, ',');
				if (tokens) {
					for ( int i = 0; *(tokens + i); i++ ) {
						s = *(tokens + i);

						if (i == 1)
							zoomLevel = atoi(s);
						else if (i == 8)
							threshold = atoi(s);
						else if (i > 1 && i < 8) {
							f[i - 2] = atof(s);
						}
						free(*(tokens + i));
					}
					free(tokens);
				}

				if (zoomLevel >= 0 && zoomLevel <= 3) {
					Kontrol_Set(zoomLevel, f, threshold);
					Kontrol_Get(zoomLevel, &pid);
					bufLen = sprintf(buf, "\r\nPID[%d]= %.5f %.5f %.5f %.5f %.5f %.5f %d\r\n",
							zoomLevel, pid.kp1, pid.kp2, pid.kd1, pid.kd2, pid.ki1, pid.ki2,
							pid.thresholdPX);

					while (ring_buffer_left(debug.TBufferTx) <= bufLen)
						HAL_Delay(1);
					serial_write_str(&debug, buf, bufLen);
				}
			}
		}

		if (HAL_GetTick() >= _countTimer) {
			_countTimer = HAL_GetTick() + 1000;

			bufLen = sprintf(buf,
					"%sRecv packets:\t(MS)%d (MA)%d (MV)%d (OC)%d (OL)%d (S)%d %d-%d TT:%d",
					vt100_lineX[11], cRecvMtrState, cRecvMtrAngle, cRecvMtrVelo, cRecvOptCam,
					cRecvOptLrf, cRecvStab, cAngleVelo[0], cAngleVelo[1], cTrackTelemetry);
			cRecvMtrState = cRecvMtrAngle = cRecvMtrVelo = cRecvOptCam = cRecvOptLrf = cRecvStab =
					cTrackTelemetry = 0;
			serial_write_str(&debug, buf, bufLen);

			bufLen = sprintf(buf, "%sSend packets:\t(MC)%d (BC)%d (SC)%d", vt100_lineX[12],
					cSendMtrCmd, cSendButtonCmd, cSendStabCmd);
			cSendMtrCmd = cSendButtonCmd = cSendStabCmd = 0;
			serial_write_str(&debug, buf, bufLen);

			char *startingLine = vt100_lineX[15];
			for ( int i = 0; i < 4; i++ ) {
				Kontrol_Get(i, &pid);
				bufLen = sprintf(buf, "%sPID[%d]= %.5f %.5f %.5f %.5f %.5f %.5f %d", startingLine,
						i, pid.kp1, pid.kp2, pid.kd1, pid.kd2, pid.ki1, pid.ki2, pid.thresholdPX);

				while (ring_buffer_left(debug.TBufferTx) <= bufLen)
					HAL_Delay(1);
				serial_write_str(&debug, buf, bufLen);

				startingLine += DEBUG_LINE_SIZE;
			}
		}

#endif	//if DEBUG==1

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	hcan1.Init.Prescaler = 5;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

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
	hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	hiwdg.Init.Reload = 250;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 89;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 89;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 999;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = TRIGGER_OFF;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */
	HAL_TIM_MspPostInit(&htim12);

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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
	huart2.Init.BaudRate = 921600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 38400;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 921600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, PC_IGNITION_Pin | WIFI_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
	 PC1 PC2 PC3 PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
			| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA5
	 PA6 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB13
	 PB14 PB3 PB4 PB5
	 PB6 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 | GPIO_PIN_14
			| GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : TRIGGER_IN_Pin */
	GPIO_InitStruct.Pin = TRIGGER_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TRIGGER_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC_PWR_CHECK_IN_Pin */
	GPIO_InitStruct.Pin = PC_PWR_CHECK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PC_PWR_CHECK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC_IGNITION_Pin */
	GPIO_InitStruct.Pin = PC_IGNITION_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PC_IGNITION_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : WIFI_EN_Pin */
	GPIO_InitStruct.Pin = WIFI_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WIFI_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : WIFI_TX_Pin WIFI_RX_Pin */
	GPIO_InitStruct.Pin = WIFI_TX_Pin | WIFI_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : WIFI_RST_Pin */
	GPIO_InitStruct.Pin = WIFI_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Begin of Functions Declaration*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	battVolt = HAL_ADC_GetValue(hadc);
}

static void CAN_Config()
{
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	/* filter can id = CAN_ID_RWS_MOTOR= 0x300 - 0x30F */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x300 << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (0x7F0 << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_OPT_x= 0x330 - 0x33F */
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x330 << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (0x7F0 << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_STAB_YPR */
	sFilterConfig.FilterBank = 2;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (canRecvStackerYPR.id << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (canRecvStackerYPR.id << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	/*##-5- Configure Transmission process #####################################*/
	can1TxHeader.StdId = 0x123;
	can1TxHeader.ExtId = 0x01;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = CAN_BUFSIZE;
	can1TxHeader.TransmitGlobalTime = DISABLE;

#if USE_CAN2
	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = (0x100 << 5);
	sFilterConfig.FilterIdLow = (0x100 << 5);
	sFilterConfig.FilterMaskIdHigh = (0x700 << 5);
	sFilterConfig.FilterMaskIdLow = (0x700 << 5);
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	can2TxHeader.StdId = 0x21;
	can2TxHeader.ExtId = 0x01;
	can2TxHeader.RTR = CAN_RTR_DATA;
	can2TxHeader.IDE = CAN_ID_STD;
	can2TxHeader.DLC = CAN_BUFSIZE;
	can2TxHeader.TransmitGlobalTime = DISABLE;

#endif	//if USE_CAN2

}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t _id = 0;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxBuffer) == HAL_OK) {
		_id = can1RxHeader.StdId;
		if (_id == canRecvOptCam.id) {
			canRecvOptCam.state = true;
			memcpy(canRecvOptCam.data, can1RxBuffer, canRecvOptCam.size);
			cRecvOptCam++;
		}
		else if (_id == canRecvOptLrf.id) {
			canRecvOptLrf.state = true;
			memcpy(canRecvOptLrf.data, can1RxBuffer, canRecvOptLrf.size);
			cRecvOptLrf++;
		}
		else if (_id == canRecvMotorState.id) {
			canRecvMotorState.state = true;
			memcpy(canRecvMotorState.data, can1RxBuffer, canRecvMotorState.size);
			cRecvMtrState++;
		}
		else if (_id == canRecvMotorAngle.id) {
			canRecvMotorAngle.state = true;
			memcpy(canRecvMotorAngle.data, can1RxBuffer, canRecvMotorAngle.size);
			cRecvMtrAngle++;
		}
		else if (_id == canRecvMotorSpeed.id) {
			canRecvMotorSpeed.state = true;
			memcpy(canRecvMotorSpeed.data, can1RxBuffer, canRecvMotorSpeed.size);
			cRecvMtrVelo++;
		}
		else if (_id == canRecvStackerYPR.id) {
			canRecvStackerYPR.state = true;
			memcpy(canRecvStackerYPR.data, can1RxBuffer, canRecvStackerYPR.size);
			cRecvStackerYpr++;
		}
	}

}

void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&pc);
}

#if DEBUG==1
void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}
#endif	//if DEBUG==1

void USART3_IRQHandler(void)
{
	USARTx_IRQHandler(&button);
}

/* TODO Non-Hardware Function Declarations*/

static long constrain(long x, long min, long max)
{
	if (x < min)
		return min;
	else if (x > max)
		return max;
	else
		return x;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float constrainf(float x, float min, float max)
{
	if (x < min)
		return min;
	else if (x > max)
		return max;
	else
		return x;
}

static float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static char** str_split(char* a_str, const char a_delim)
{
	char** result = 0;
	size_t count = 0;
	char* tmp = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp) {
		if (a_delim == *tmp) {
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = malloc(sizeof(char*) * count);

	if (result) {
		size_t idx = 0;
		char* token = strtok(a_str, delim);

		while (token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return result;
}

static void buttonPanelUpdate(uint32_t millis, uint32_t btn)
{
	static uint8_t _lrfCounter = 0;

	/* MOVEMENT SPEED*/
	manualSpeedMaxIndex = btn & 0b11;
	if (manualSpeedMaxIndex > 2)
		manualSpeedMaxIndex = 2;

	/* FIRING MODE
	 * 0: inf
	 * 1: 100ms
	 * 2: 1s
	 */
	fireState = (btn >> 8) & 0b11;

	/* CAMERA SELECT
	 * 0: sony
	 * 1: thermal
	 */
	bitWrite(canSendButton.data[1], 0, bitRead(btn,12));

	/* ZOOM
	 * 0: none
	 * 1: zoom in
	 * 2: zoom out
	 */
	/* APERTURE (EDGE ENHANCEMENT)
	 * 1: aperture up
	 * 2: aperture down
	 */
	bitWrite(canSendButton.data[1], 5, bitRead(btn,6));
	if (!bitRead(btn, 6))
		bitWrite(canSendButton.data[1], 6, bitRead(btn,7));
	else
		bitClear(canSendButton.data[1], 6);

	/* FOCUS
	 * 0: none
	 * 1: focus far
	 * 2: focus near
	 */
	bitWrite(canSendButton.data[1], 3, bitRead(btn,10));
	bitWrite(canSendButton.data[1], 4, bitRead(btn,11));

	/* LRF */
	bitWrite(canSendButton.data[2], 0, bitRead(btn,2));
	bitWrite(canSendButton.data[2], 1, bitRead(btn,3));

	if (!bitRead(btn, 2) && bitRead(jRight.dtab, 2))
		bitSet(canSendButton.data[2], 3);
	else
		bitClear(canSendButton.data[2], 3);

//	if (bitRead(btn, 3))
//		lrfData.distance = 0;

	/* LRF manual
	 * 0: none
	 * 1: lrf manual up
	 * 2: lrf manual down
	 */
	if (bitRead(btn, 4)) {
		if (_lrfCounter == 0) {
			lrfData.distance /= 50;
			if (lrfData.distance > 0)
				lrfData.distance = (lrfData.distance * 50) - 50;
		}
		if (++_lrfCounter > 4)
			_lrfCounter = 0;
	}
	else if (bitRead(btn, 5)) {
		if (_lrfCounter == 0) {
			lrfData.distance /= 50;
			lrfData.distance = (lrfData.distance * 50) + 50;
		}
		if (++_lrfCounter > 4)
			_lrfCounter = 0;
	}

	else {
		_lrfCounter = 0;
	}

	/* Target select
	 * bit0: next target
	 * bit1: prev target
	 */
	if (bitRead(btn, 13)) {
		trackSelectTarget = TRK_SEL_PREV;
	}
	else if (bitRead(btn, 14)) {
		trackSelectTarget = TRK_SEL_NEXT;
	}
	else {
		trackSelectTarget = TRK_SEL_NONE;
	}

	/* TRIGGER HOT */
	bitWrite(canSendButton.data[0], 0, bitRead(btn, 15));

}

static void buttonJoystickUpdate(uint32_t millis)
{
	uint8_t _u8;
	static uint8_t fireTriggerState = 0;
	static uint32_t fireTimer = 0;
#if MOVEMENT_CHANGE_ENABLE==1
	static uint8_t jRightButton = 0;
	static uint8_t jLeftButton = 0;
#endif	//if MOVEMENT_CHANGE_ENABLE==1

	/* Button from Joystick */
	/* ZOOM */
	_u8 = (canSendButton.data[1] >> 1) & 0b11;
	/* zoomState= zoom in */
	if (_u8 == 1) {
		if (jLeft.elevation < 400) {
			/* set to zoom none */
			bitClear(canSendButton.data[1], 1);
			bitClear(canSendButton.data[1], 2);
		}
	}
	/* zoomState= zoom out */
	else if (_u8 == 2) {
		if (jLeft.elevation > -400) {
			/* set to zoom none */
			bitClear(canSendButton.data[1], 1);
			bitClear(canSendButton.data[1], 2);
		}
	}
	/* zoomState= zoom none */
	else {
		/* set to zoom in */
		if (jLeft.elevation > 600 && trackGateResize == TRK_GATE_NONE) {
			bitSet(canSendButton.data[1], 1);
			bitClear(canSendButton.data[1], 2);
		}
		/* set to zoom out */
		else if (jLeft.elevation < -600 && trackGateResize == TRK_GATE_NONE) {
			bitClear(canSendButton.data[1], 1);
			bitSet(canSendButton.data[1], 2);
		}
	}

	/* GATE TRACK RESIZE */
	/* trackGateResize = bigger */
	if (trackGateResize == TRK_GATE_BIGGER) {
		if (jLeft.azimuth < 400)
			trackGateResize = TRK_GATE_NONE;
	}
	/* trackGateResize = smaller */
	else if (trackGateResize == TRK_GATE_SMALLER) {
		if (jLeft.azimuth > -400)
			trackGateResize = TRK_GATE_NONE;
	}
	/* trackGateResize = none */
	else {
		_u8 = canSendButton.data[1] & 0b110;
		if ((jLeft.azimuth > 600) && (_u8 == 0))
			trackGateResize = TRK_GATE_BIGGER;
		else if ((jLeft.azimuth < -600) && (_u8 == 0))
			trackGateResize = TRK_GATE_SMALLER;
	}

	/* FIRING */
	if (fireTriggerState > 0) {
		if (fireTimer > 0 && millis >= fireTimer) {
			WEAPON_FIRE_OFF();
			fireTimer = 0;
		}
	}

	if (canRecvMotorState.online
			&& bitRead(canRecvMotorState.data[1], 0) && !bitRead(canRecvMotorState.data[1], 1)) {
		if ((jRight.dtab & 0b11) == 0b11) {
			if (fireTriggerState == 0) {
				if (fireState == ST_FIRE_COUNT_1)
					if (WeaponType == Weapon_762)
						fireTimer = millis + (TIME_FIRE_COUNT_1 * 3);
					else
						fireTimer = millis + TIME_FIRE_COUNT_1;
				else if (fireState == ST_FIRE_COUNT_3)
					if (WeaponType == Weapon_762)
						fireTimer = millis + TIME_FIRE_COUNT_3 * 10;
					else
						fireTimer = millis + TIME_FIRE_COUNT_3;
				else
					fireTimer = 0;

				WEAPON_FIRE_ON();
				fireTriggerState = 1;
			}
		}
		else {
			WEAPON_FIRE_OFF();
			fireTimer = 0;
			fireTriggerState = 0;
		}
	}
	else {
		WEAPON_FIRE_OFF();
		fireTimer = 0;
		fireTriggerState = 0;
	}

	/* MOVEMENT BUTTON */
#if MOVEMENT_CHANGE_ENABLE==1
	/* TRAVEL & STAB change */
	if (bitRead(jRight.dtab, 3) && !bitRead(jRightButton, 3)) {
		if (!bitRead(movementMode, MOVE_MEMORY_bit) && !bitRead(movementMode, MOVE_TRACK_bit)) {
			if (!bitRead(movementMode, MOVE_STAB_bit))
				motorStabStarting(millis);
			else
				motorJoystickStarting(millis);
		}
	}
	bitWrite(jRightButton, 3, bitRead(jRight.dtab,3));

	/* TRACK change */
	if (bitRead(jLeft.dtab, 3) && !bitRead(jLeftButton, 3)) {
		if (!bitRead(movementMode, MOVE_MEMORY_bit)) {
			if (!bitRead(movementMode, MOVE_TRACK_bit))
				motorTrackingStarting(millis);
			else {
				if (bitRead(movementMode, MOVE_STAB_bit))
					motorStabStarting(millis);
				else
					motorJoystickStarting(millis);
			}
		}
	}
	bitWrite(jLeftButton, 3, bitRead(jLeft.dtab,3));

	/* MEMORY change */
	if (bitRead(jLeft.dtab, 2) && !bitRead(jLeftButton, 2)) {
		motorMemoryStarting(millis);
	}
	bitWrite(jLeftButton, 2, bitRead(jLeft.dtab,2));
#endif	//if MOVEMENT_CHANGE_ENABLE==1
}

static void buttonHandler()
{
	uint32_t millis = HAL_GetTick();
	static uint32_t sendTimer = 1000;
	static uint32_t btnLed0Timer = 0;
	char c;
	char *s;
	char **tokens;
	bool btnCompleted = false;
	uint32_t _btnState = 0;
	int16_t _i16;
	const float alphaRight = 0.5;
	const float alphaLeft = 0.5;
	static int jsRight[2] = { 0, 0 };
	static int jsLeft[2] = { 0, 0 };

	static uint32_t lastReceivedTimer = 5000;
	static uint8_t dataAvailable = 0;
	static uint32_t updateUnavailableDataTimer = 0;

#if DEBUG_BUTTON==1
	const uint8_t startDebugLine = 3;
#endif	//if DEBUG_BUTTON==1

	/* RECEIVER */
	if (serial_available(&button)) {
		c = serial_read(&button);

		if (c == '$')
			memset(bufButton, 0, BUTTON_BUFSIZE);
		else if (c == '*')
			btnCompleted = true;

		strncat(bufButton, (const char *) &c, 1);
	}

	if (btnCompleted) {
		lastReceivedTimer = millis;
		dataAvailable = 1;
		s = strstr(bufButton, "$BTN,");
		if (s) {
#if DEBUG_BUTTON==1
			bufLen = sprintf(buf, "%s%s", vt100_lineX[startDebugLine], s);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUTTON==1
			tokens = str_split(bufButton, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					switch (i)
					{
					case 1:
						_btnState = atoi(s);
						break;
					case 2:
						jRight.dtab = atoi(s);
						break;
					case 3:
						_i16 = atoi(s) - 885;
						jsRight[0] = (int) (((1.0 - alphaRight) * (float) jsRight[0])
								+ (alphaRight * (float) _i16));
						break;
					case 4:
						_i16 = atoi(s) - 875;
						jsRight[1] = (int) (((1.0 - alphaRight) * (float) jsRight[1])
								+ (alphaRight * (float) _i16));
						break;
					case 5:
						jLeft.dtab = atoi(s);
						break;
					case 6:
						_i16 = atoi(s) - 1120;
						jsLeft[0] = (int) (((1.0 - alphaLeft) * (float) jsLeft[0])
								+ (alphaLeft * (float) _i16));
						break;
					case 7:
						_i16 = atoi(s) - 1120;
						jsLeft[1] = (int) (((1.0 - alphaLeft) * (float) jsLeft[1])
								+ (alphaLeft * (float) _i16));
						break;
					}

					free(*(tokens + i));
				}
				free(tokens);
			}

			int _i;
			float _f;
			const int32_t out_min = -900;
			const int32_t out_max = 900;
			const float maxValue = joystickMovementMaxValue;
			/* convert azimuth & elevation of RIGHT Joystick */
			_i = (int) constrain(map((long) jsRight[0], -250, 230, out_min, out_max), out_min,
					out_max);
			/* if below 30 degrees */
			if (abs(_i) < 300)
				_i /= 5;
			_f = (float) _i / 10.0;
			jRight.azimuth = (int) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
			if (_f < 0)
				jRight.azimuth = 0 - jRight.azimuth;

			_i = (int) constrain(map((long) jsRight[1], -200, 280, out_min, out_max), out_min,
					out_max);
			/* if below 30 degrees */
			if (abs(_i) < 300)
				_i /= 5;
			_f = (float) _i / 10.0;
			jRight.elevation = (int) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
			if (_f < 0)
				jRight.elevation = 0 - jRight.elevation;

			/* convert azimuth & elevation of LEFT Joystick */
			_i = (int16_t) constrain(map((long) jsLeft[0], -180, 180, out_min, out_max), out_min,
					out_max);
			/* if below 30 degrees */
			if (abs(_i) < 300)
				_i /= 5;
			_f = (float) _i / 10.0;
			jLeft.azimuth = (int) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
			if (_f < 0)
				jLeft.azimuth = 0 - jLeft.azimuth;

			_i = (int16_t) constrain(map((long) jsLeft[1], -180, 175, out_min, out_max), out_min,
					out_max);
			/* if below 30 degrees */
			if (abs(_i) < 300)
				_i /= 5;
			_f = (float) _i / 10.0;
			jLeft.elevation = (int) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
			if (_f < 0)
				jLeft.elevation = 0 - jLeft.elevation;

#if DEBUG_BUTTON==1
			bufLen = sprintf(buf, "%s0x%04X\t%d:%d:%d\t%d:%d:%d", vt100_lineX[startDebugLine + 1],
					(int) _btnState, jRight.dtab, jRight.azimuth, jRight.elevation, jLeft.dtab,
					jLeft.azimuth, jLeft.elevation);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUTTON==1

			buttonPanelUpdate(millis, _btnState);
		}
	} /* if (btnCompleted) { */

	if (millis >= lastReceivedTimer + 200) {
		/* clear all data */
		if (dataAvailable == 1) {
			updateUnavailableDataTimer = 0;
			dataAvailable = 0;
		}

		if (millis >= updateUnavailableDataTimer) {
			updateUnavailableDataTimer = millis + 50;
			_btnState = 0;
			jLeft.dtab = 0;
			jLeft.azimuth = jLeft.elevation = 0;
			jRight.dtab = 0;
			jRight.azimuth = jRight.elevation = 0;

			buttonPanelUpdate(millis, _btnState);
		}
	}

	buttonJoystickUpdate(millis);

	/* TRANSMITTER */
	if (millis >= sendTimer) {
		sendTimer = millis + 50;

		/* motorControl & optroniks online & battVolt >23V*/
		if (canRecvMotorState.online && canRecvOptCam.online && canRecvOptLrf.online
				&& battVoltActual(battVolt) >= 22000)
			bitSet(buttonLed, 0);
		else {
			if (millis >= btnLed0Timer) {
				if (battVoltActual(battVolt) < 23000)
					btnLed0Timer = millis + 100;
				else if (!canRecvMotorState.online)
					btnLed0Timer = millis + 1000;
				else {
					if (!bitRead(buttonLed, 0))
						btnLed0Timer = millis + 5000;
					else
						btnLed0Timer = millis + 1000;
				}
				bitWrite(buttonLed, 0, !bitRead(buttonLed,0));
			}
		}

		bufLen = sprintf(buf, "$LED,%d*", buttonLed);
		serial_write_str(&button, buf, bufLen);

		can1TxHeader.StdId = canSendButton.id;
		memcpy(can1TxBuffer, canSendButton.data, canSendButton.size);
		can1TxHeader.DLC = canSendButton.size;
		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK)
			cSendButtonCmd++;
		else
			sendTimer = millis + 1;
	}

}

static void canRecvHandler()
{
	uint32_t millis = HAL_GetTick();
#if DEBUG_BUS==1 || DEBUG_CAM==1
	const uint8_t startLineDebug = 10;
#endif	//if DEBUG_BUS==1

	static uint32_t recvOptCamTimer = 0;
	static uint32_t recvOptLrfTimer = 0;
	static uint32_t recvMotorTimer = 0;
	uint8_t _u8, _zoomLevel;
	uint16_t lrfVal;

	/* Receive Data from Optronik - CAM */
	if (canRecvOptCam.state) {
		canRecvOptCam.state = false;
		recvOptCamTimer = millis + CAN_RECEIVE_TIMEOUT;
		canRecvOptCam.online = 1;

		_u8 = _zoomLevel = (canRecvOptCam.data[0] >> 1) & 0b11;

		if (_zoomLevel > 2)
			_zoomLevel = 2;

		/* if CAM==CAM_ST_THERMAL */
		if (bitRead(canRecvOptCam.data[0], 0)) {
			if (zoomValueFromOptCam != _u8) {
				/* adjust crosshair */
				crossAdjust(0, 0);
			}
		}
		/* if CAM==CAM_ST_SONY */
		else {
			if (zoomValueFromOptCam != _u8) {
				/* adjust crosshair */
				crossAdjust(xCross[_u8], yCross[_u8]);
			}
		}
		zoomValueFromOptCam = _u8;

#if DEBUG_BUS==1 || DEBUG_CAM==1
		bufLen = sprintf(buf, "%sRecv Opt Cam: ", vt100_lineX[startLineDebug + 0]);
		serial_write_str(&debug, buf, bufLen);
		for ( int i = 0; i < canRecvOptCam.size; i++ ) {
			bufLen = sprintf(buf, "0x%02X ", canRecvOptCam.data[i]);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if DEBUG_BUS==1 || DEBUG_CAM==1
	}

	if (recvOptCamTimer > 0 && millis >= recvOptCamTimer) {
		recvOptCamTimer = 0;
		canRecvOptCam.online = 0;

		memset(canRecvOptCam.data, 0, canRecvOptCam.size);
#if DEBUG_BUS==1 || DEBUG_CAM==1
		bufLen = sprintf(buf, "%s", vt100_lineX[startLineDebug + 0]);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUS==1 || DEBUG_CAM==1

	}

	/* Receive Data from Optronik - LRF */
	if (canRecvOptLrf.state) {
		canRecvOptLrf.state = false;
		recvOptLrfTimer = millis + CAN_RECEIVE_TIMEOUT;
		canRecvOptLrf.online = 1;

		lrfVal = (uint16_t) canRecvOptLrf.data[2] << 8 | canRecvOptLrf.data[1];

		if (canRecvOptLrf.data[0] != lrfData.counter) {
			lrfData.counter = canRecvOptLrf.data[0];
			lrfData.distance = lrfVal;
		}

#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%sRecv Opt LRF: ", vt100_lineX[startLineDebug + 2]);
		serial_write_str(&debug, buf, bufLen);
		for ( int i = 0; i < canRecvOptLrf.size; i++ ) {
			bufLen = sprintf(buf, "0x%02X ", canRecvOptLrf.data[i]);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if DEBUG_BUS==1

	}

	if (recvOptLrfTimer > 0 && millis >= recvOptLrfTimer) {
		recvOptLrfTimer = 0;
		canRecvOptLrf.online = 0;

		memset(canRecvOptLrf.data, 0, canRecvOptLrf.size);
#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%s", vt100_lineX[startLineDebug + 2]);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUS==1

	}

	/* Receive Data from MotorControl */
	if (canRecvMotorState.state) {
		canRecvMotorState.state = false;
		recvMotorTimer = millis + CAN_RECEIVE_TIMEOUT;
		canRecvMotorState.online = 1;

		if ((canRecvMotorState.data[1] & 0b11) == 0b01)
			bitSet(buttonLed, 1);
		else
			bitClear(buttonLed, 1);

		cAngleVelo[0] = ((uint16_t) canRecvMotorState.data[3] << 8) | canRecvMotorState.data[2];
		cAngleVelo[1] = ((uint16_t) canRecvMotorState.data[5] << 8) | canRecvMotorState.data[4];

#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%sRecv Motor: ", vt100_lineX[startLineDebug + 3]);
		serial_write_str(&debug, buf, bufLen);
		for ( int i = 0; i < canRecvMotorState.size; i++ ) {
			bufLen = sprintf(buf, "0x%02X ", canRecvMotorState.data[i]);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if DEBUG_BUS==1

	}

	if (recvMotorTimer > 0 && millis >= recvMotorTimer) {
		recvMotorTimer = 0;
		canRecvMotorState.online = 0;

		memset(canRecvMotorState.data, 0, canRecvMotorState.size);
#if DEBUG_BUS==1
		bufLen = sprintf(buf, "%s", vt100_lineX[startLineDebug + 3]);
		serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_BUS==1

	}

}

static uint16_t battVoltActual(uint8_t v)
{
	return constrain(map(v, 0, 166, 0, 23890), 0, 50000);
}

static void crossAdjust(int x, int y)
{
	cross.x = x;
	cross.y = y;
	cross.state = 1;
	sprintf(pcUpdateBuffer.crossBuffer, "$CROSS,%d,%d*", x, y);
}

static void movementAdjust()
{
	if (bitRead(movementMode, MOVE_MEMORY_bit))
		sprintf(pcUpdateBuffer.moveBuffer, "$DISPSTR,1,5,30,MEMORY MODE*");
	else if (bitRead(movementMode, MOVE_TRACK_bit))
		sprintf(pcUpdateBuffer.moveBuffer, "$DISPSTR,1,5,30,TRACK MODE*");
	else if (bitRead(movementMode, MOVE_STAB_bit))
		sprintf(pcUpdateBuffer.moveBuffer, "$DISPSTR,1,5,30,STAB MODE*");
	else if (bitRead(movementMode, MOVE_TRAVEL_bit))
		sprintf(pcUpdateBuffer.moveBuffer, "$DISPSTR,1,5,30,TRAVEL MODE*");
	else
		sprintf(pcUpdateBuffer.moveBuffer, "$DISPSTR,1,5,30,MANUAL MODE*");
}

static void graticuleAdjust()
{
	uint8_t _u8 = 0;

	if (!bitRead(canRecvOptCam.data[0], 0)) {
		if (zoomValueFromOptCam == 0) {
			sprintf(pcUpdateBuffer.fovBuffer, "$DISPSTR,0,5,5,Observe FOV*");
			_u8 = 0;
			if (WeaponType == Weapon_1270) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW1270_300m[_u8][0], GratW1270_300m[_u8][1], GratW1270_500m[_u8][0],
						GratW1270_500m[_u8][1], GratW1270_1000m[_u8][0], GratW1270_1000m[_u8][1]);
			}
			else if (WeaponType == Weapon_762) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW762_300m[_u8][0], GratW762_300m[_u8][1], GratW762_500m[_u8][0],
						GratW762_500m[_u8][1], GratW762_1000m[_u8][0], GratW762_1000m[_u8][1]);
			}
		}
		else if (zoomValueFromOptCam == 1) {
			sprintf(pcUpdateBuffer.fovBuffer, "$DISPSTR,0,5,5,Combat FOV*");
			_u8 = 1;
			if (WeaponType == Weapon_1270) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW1270_300m[_u8][0], GratW1270_300m[_u8][1], GratW1270_500m[_u8][0],
						GratW1270_500m[_u8][1], GratW1270_1000m[_u8][0], GratW1270_1000m[_u8][1]);
			}
			else if (WeaponType == Weapon_762) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW762_300m[_u8][0], GratW762_300m[_u8][1], GratW762_500m[_u8][0],
						GratW762_500m[_u8][1], GratW762_1000m[_u8][0], GratW762_1000m[_u8][1]);
			}
		}
		else {
			sprintf(pcUpdateBuffer.fovBuffer, "$DISPSTR,0,5,5,Near FOV*");
			_u8 = 2;
			if (WeaponType == Weapon_1270) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW1270_300m[_u8][0], GratW1270_300m[_u8][1], GratW1270_500m[_u8][0],
						GratW1270_500m[_u8][1], GratW1270_1000m[_u8][0], GratW1270_1000m[_u8][1]);
			}
			else if (WeaponType == Weapon_762) {
				sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
						GratW762_300m[_u8][0], GratW762_300m[_u8][1], GratW762_500m[_u8][0],
						GratW762_500m[_u8][1], GratW762_1000m[_u8][0], GratW762_1000m[_u8][1]);
			}
		}
	}
	else {
		sprintf(pcUpdateBuffer.fovBuffer, "$DISPSTR,0,5,5,Combat FOV*");
		_u8 = 1;
		if (WeaponType == Weapon_1270) {
			sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
					GratW1270_300m[_u8][0], GratW1270_300m[_u8][1], GratW1270_500m[_u8][0],
					GratW1270_500m[_u8][1], GratW1270_1000m[_u8][0], GratW1270_1000m[_u8][1]);
		}
		else if (WeaponType == Weapon_762) {
			sprintf(pcUpdateBuffer.graticuleBuffer, "$GRAT,%d,%d,%d,%d,%d,%d*",
					GratW762_300m[_u8][0], GratW762_300m[_u8][1], GratW762_500m[_u8][0],
					GratW762_500m[_u8][1], GratW762_1000m[_u8][0], GratW762_1000m[_u8][1]);
		}
	}
}

static void pcHandler()
{
	uint32_t millis = HAL_GetTick();
	uint8_t _u8;
	char c;
	bool trkCompleted = false;
	char *s;
	char **tokens;
	static int8_t _trkSelTarget = TRK_SEL_NONE;
	static uint32_t trkLiveTimer = 0;
	static uint32_t sendTimer = 5000;
	static uint32_t moveTimer = 1000;
	static uint32_t displayTimer = 1000;
	static uint32_t displaySizeTimer = 10000;

	char updateBuf[UART_BUFSIZE];

	bool cmdFound = false;
#if DEBUG_PC==1
	const uint8_t startDebugLine = 15;
#endif	//if DEBUG_PC==1

	if (millis >= sendTimer) {
		sendTimer = millis + 100;

		if (lrfData.distance < 30 && lrfData.distance > 0)
			sprintf(updateBuf, "$LRVAL,Error:%d*$AZVAL,%.2f*$ELVAL,%.2f*", lrfData.distance, ypr[0],
					ypr[1]);
		else
			sprintf(updateBuf, "$LRVAL,%d m*$AZVAL,%.2f*$ELVAL,%.2f*", lrfData.distance, ypr[0],
					ypr[1]);

		if (millis >= displaySizeTimer) {
			displaySizeTimer = millis + 10000;
			sprintf(buf, "$SWCAM,1*%s", updateBuf);
			memcpy(updateBuf, buf, strlen(buf));
		}

		if (millis >= moveTimer) {
			moveTimer = millis + 500;

			movementAdjust();
			sprintf(buf, "%s%s", pcUpdateBuffer.moveBuffer, updateBuf);
			memcpy(updateBuf, buf, strlen(buf));
		}

		if (millis >= displayTimer) {
			displayTimer = millis + 500;

			graticuleAdjust();
			sprintf(buf, "%s%s%s", pcUpdateBuffer.graticuleBuffer, pcUpdateBuffer.fovBuffer,
					updateBuf);
			memcpy(updateBuf, buf, strlen(buf));
		}

		sprintf(buf, "$GATE,%d*%s", trackGateResize, updateBuf);
		memcpy(updateBuf, buf, strlen(buf));

		if (trackSelectTarget != _trkSelTarget) {
			if ((trackSelectTarget == TRK_SEL_NEXT) || (trackSelectTarget == TRK_SEL_PREV)) {
				sprintf(buf, "$TRKSEL,%d*%s", trackSelectTarget, updateBuf);
				memcpy(updateBuf, buf, strlen(buf));
			}

			_trkSelTarget = trackSelectTarget;
		}

		if (cross.state == 1) {
			sprintf(buf, "%s%s", pcUpdateBuffer.crossBuffer, updateBuf);
			memcpy(updateBuf, buf, strlen(buf));
			cross.state = 0;
		}

		serial_write_str(&pc, updateBuf, strlen(updateBuf));
	}

	if (trkLiveTimer > 0 && millis >= trkLiveTimer) {
		/* no data from tracker */
		trkLiveTimer = 0;
		trackerData.activeID = 0;
		trackerData.trkX = 0;
		trackerData.trkY = 0;
		/* TODO Potential problem arise when tracking lose all target, PID stop*/
		trackingPidDeInit();
	}

	if (serial_available(&pc)) {
		c = serial_read(&pc);
		if (c == '$')
			memset(trackerData.buf, 0, TRK_BUFSIZE);
		else if (c == '*')
			trkCompleted = true;

		sprintf(trackerData.buf, "%s%c", trackerData.buf, c);
	}

	if (trkCompleted) {
		if (!cmdFound) {
			s = strstr(trackerData.buf, "$TRKUD,");
#if DEBUG_PC==1
			bufLen = sprintf(buf, "%s\r\n", s);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_PC==1
			if (s) {
				cmdFound = true;
				tokens = str_split(trackerData.buf, ',');
				if (tokens) {
					for ( int i = 0; *(tokens + i); i++ ) {
						s = *(tokens + i);
						if (i == 1)
							trackerData.activeID = atoi(s);
						else if (i == 2)
							trackerData.trkX = atoi(s);
						else if (i == 3) {
							trackerData.trkY = atoi(s);
						}

						free(*(tokens + i));
					}
					free(tokens);
					trkLiveTimer = millis + 1000;
					/* re-init PID when target re-appear */
					trackingPidInit();
				}	//if (tokens)
				cTrackTelemetry++;
			}	//if (s)
		}

		if (!cmdFound) {
			s = strstr(trackerData.buf, "$CALI,");
			if (s) {
				if (trackerData.buf[6] == '*')
					_u8 = trackerData.buf[5];
				if (_u8 == '0')
					WeaponType = Weapon_1270;
				else if (_u8 == '1')
					WeaponType = Weapon_762;
			}	//if (s)
		}

		if (!cmdFound) {
			s = strstr(trackerData.buf, "$DATA,");
			if (s) {
				cmdFound = true;
#if DEBUG==1
				bufLen = sprintf(buf, "%s%s", vt100_lineX[10], s);
				serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG==1
				tokens = str_split(trackerData.buf, ',');
				if (tokens) {
					for ( int i = 0; *(tokens + i); i++ ) {
						s = *(tokens + i);
						if (i == 1)
							WeaponType = atoi(s);
//						else if(i==2);
//							bufLen=sprintf()

						free(*(tokens + i));
					}
					free(tokens);
				}	//if (tokens)
			}
			//if (s)
		}

	}	//if (trkCompleted)

}

static void motorUpdateData(uint8_t enable, float pan, float tilt)
{
	if (enable > 0)
		motorEnable();
	else
		motorDisable();

	Union_u az, el;
	az.f = pan;
	el.f = tilt;
	for ( int i = 0; i < 4; i++ ) {
		canSendMotorCommand.data[i] = az.b[i];
		canSendMotorCommand.data[i + 4] = el.b[i];
	}
}

static void motorHandler()
{
	static uint32_t sendTimer = 1000;
	static uint32_t stabModuleTimer = 2000;

	panMoveMax = MTR_PAN_SPEED_DEG[manualSpeedMaxIndex];
	tiltMoveMax = MTR_TILT_SPEED_DEG[manualSpeedMaxIndex];

#if MOVEMENT_CHANGE_ENABLE==1
	if (bitRead(movementMode, MOVE_MEMORY_bit))
		motorMemoryHandler(HAL_GetTick());
	else if (bitRead(movementMode, MOVE_TRACK_bit))
		motorTrackingHandler(HAL_GetTick());
	else if (bitRead(movementMode, MOVE_STAB_bit))
		motorStabHandler(HAL_GetTick());
	else
		motorJoystickHandler(HAL_GetTick());
#else
	motorJoystickHandler(millis);
#endif	//if MOVEMENT_CHANGE_ENABLE==1

	if (HAL_GetTick() >= sendTimer) {
		sendTimer = HAL_GetTick() + MOTOR_UPDATE_TIMEOUT;

		can1TxHeader.StdId = canSendMotorCommand.id;
		memcpy(can1TxBuffer, canSendMotorCommand.data, canSendMotorCommand.size);
		can1TxHeader.DLC = canSendMotorCommand.size;
		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK)
			cSendMtrCmd++;
		else
			sendTimer = HAL_GetTick() + 1;

	}

	if (HAL_GetTick() >= stabModuleTimer) {
		stabModuleTimer = HAL_GetTick() + 100;

		can1TxHeader.StdId = canSendStabMode.id;
		memcpy(can1TxBuffer, canSendStabMode.data, canSendStabMode.size);
		can1TxHeader.DLC = canSendStabMode.size;
		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK)
			cSendStabCmd++;
		else
			stabModuleTimer = HAL_GetTick() + 1;
	}

}

static float motorJoystickConvert(int input, float _spdMax)
{
	float ret = 0;
	float out_min, out_max;

	if (_spdMax < 0)
		_spdMax = 0 - _spdMax;

	out_min = 0 - _spdMax;
	out_max = _spdMax;
	ret = constrainf(
			mapf(input, (0.0f - joystickMovementMaxValue), joystickMovementMaxValue, out_min,
					out_max), out_min, out_max);

	return ret;
}

static void motorJoystickHandler(uint32_t millis)
{
	static uint32_t updateTimer = 1000;
	uint8_t _enable = 0;
	float _pan = 0;
	float _tilt = 0;

	if (millis >= updateTimer) {
		updateTimer = millis + 100;

		if (bitRead(jRight.dtab, 0)) {
			_enable = 0b11;

			_pan = motorJoystickConvert(jRight.azimuth, panMoveMax);
			_tilt = motorJoystickConvert(jRight.elevation, tiltMoveMax);

			motorUpdateData(_enable, _pan, _tilt);
		}
		else
			motorUpdateData(0, 0, 0);
	}
}

#if MOVEMENT_CHANGE_ENABLE==1
static void motorJoystickStarting(uint32_t millis)
{
	trackingPidDeInit();
	stabPidDeInit(0);
	bitClear(movementMode, MOVE_TRAVEL_bit);
	bitClear(movementMode, MOVE_STAB_bit);
	bitClear(movementMode, MOVE_TRACK_bit);
	bitClear(movementMode, MOVE_MEMORY_bit);
}

static void stabPidInit()
{
	stabilizeMode = MOVEMENT_MODE_ON;
	bitSet(canSendStabMode.data[0], 0);

	motorEnable();
}

static void stabPidDeInit()
{
	stabilizeMode = MOVEMENT_MODE_OFF;
	bitClear(canSendStabMode.data[0], 0);
	bitClear(movementMode, MOVE_STAB_bit);

	motorDisable();
}

static void motorStabHandler(uint32_t millis)
{
	motorEnable();
}

static void motorStabStarting(uint32_t millis)
{
	bitSet(movementMode, MOVE_STAB_bit);
	stabPidInit();
}

static void trackingPidInit()
{
	trackingMode = MOVEMENT_MODE_ON;
	Kontrol_init();
	motorEnable();
}

static void trackingPidDeInit()
{
	trackingMode = MOVEMENT_MODE_OFF;
	bitClear(movementMode, MOVE_TRACK_bit);
	motorDisable();
}

static void motorTrackingHandler(uint32_t millis)
{
	static uint32_t updateMotorTimer = 0;
	uint8_t _u8 = 0;

	if (millis >= updateMotorTimer) {
		updateMotorTimer = millis + 100;

		motorEnable();

		if (!bitRead(canRecvOptCam.data[0], 0)) {
			_u8 = zoomValueFromOptCam;
			if (_u8 > 2)
				_u8 = 2;
		}

		/* convert to motor value */
		if (trackingMode == MOVEMENT_MODE_ON) {
			/* TODO add tracking control here*/
			float wMotor[2] = { 0.0f, 0.0f };
			Kontrol_CalcQDot(_u8, motorPosActual[0], motorPosActual[1], motorVeloActual[0],
					motorVeloActual[1], trackerData.trkX, trackerData.trkY, &wMotor[0], &wMotor[1]);

			Union_u _az, _el;
			_az.f = wMotor[0];
			_el.f = wMotor[1];
			for ( int i = 0; i < 4; i++ ) {
				canSendTrackCorrection.data[i] = _az.b[i];
				canSendTrackCorrection.data[i + 4] = _el.b[i];
			}

#if DEBUG_TRACK==1
			bufLen = sprintf(buf, "%smode=%d\tx= %d y=%d", vt100_lineX[9], trackingMode,
					(int) trackerData.trkX, (int) trackerData.trkY);
			serial_write_str(&debug, buf, bufLen);
			bufLen = sprintf(buf, "%span= %.3f tilt=%.3f", vt100_lineX[10], wMotor[0], wMotor[0]);
			serial_write_str(&debug, buf, bufLen);
#endif	//if DEBUG_TRACK==1
		}
	}
}

static void motorTrackingStarting(uint32_t millis)
{
	stabPidDeInit();
	bitSet(movementMode, MOVE_TRACK_bit);
	trackingPidInit();
}

static void motorMemoryHandler(uint32_t millis)
{
	bitClear(movementMode, MOVE_MEMORY_bit);
}

static void motorMemoryStarting(uint32_t millis)
{
	bitSet(movementMode, MOVE_MEMORY_bit);
}
#endif	//if MOVEMENT_CHANGE_ENABLE==1

/* TODO End of Functions Declaration*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
