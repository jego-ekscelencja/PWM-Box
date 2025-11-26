/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "lptim.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	EDGE_OUTPUT_TIM2_CH3 = 0,   //PA2, TIM2_CH3
	EDGE_OUTPUT_LPTIM1_OUT      // PB2, LPTIM1_OUT
} EdgeOutput_t;

typedef enum {
	EDGE_SLEW_LOW = 0, EDGE_SLEW_MEDIUM, EDGE_SLEW_HIGH, EDGE_SLEW_VERY_HIGH
} EdgeSlew_t;

typedef struct {
	uint8_t mode;       // 0-7  LowEMI-Stop, LowEMI-Sleep, CA-Base
	uint8_t freq_idx;   // 0-15 ustawienie częstotliwości
	uint8_t duty_idx;   // 0-7 ustawienie wypełnienia
	EdgeSlew_t edge_slew;  // LOW / MEDIUM / HIGH / VERY_HIGH
} PwmBoxJumpers_t;

#define HAL_ADC_MODULE_ENABLED
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Kody trybów pracy
#define PWMBOX_MODE_LOWEMI_STOP         0u
#define PWMBOX_MODE_LOWEMI_SLEEP        1u
#define PWMBOX_MODE_CA_BASE             2u
#define PWMBOX_MODE_CA_PRBS             3u
#define PWMBOX_MODE_CA_DMA              4u
#define PWMBOX_MODE_AUTOTUNE_FREQ       5u
#define PWMBOX_MODE_AUTOTUNE_SPREAD     6u
#define PWMBOX_MODE_FIXED_TONE          7u
// - - ADC
#define HAL_ADC_MODULE_ENABLED
#define ISENSE_ADC_INSTANCE   ADC1
#define ISENSE_ADC_CHANNEL    ADC_CHANNEL_0

// Tabela częstotliwości  w hz
static const float s_freq_table[16] = { 0.25f,   // 0000
		0.5f,    // 0001
		1.0f,    // 0010
		2.0f,    // 0011
		4.0f,    // 0100
		8.0f,    // 0101
		16.0f,   // 0110
		32.0f,   // 0111
		64.0f,   // 1000
		100.0f,  // 1001
		150.0f,  // 1010
		200.0f,  // 1011
		300.0f,  // 1100
		500.0f,  // 1101
		800.0f,  // 1110
		1000.0f  // 1111
		};

// Tabela wypełnień
static const float s_duty_table[8] = { 0.50f,
		0.10f,  // 001  10%
		0.25f,  // 010  25%
		0.33f,  // 011  33%
		0.66f,  // 100  66%
		0.75f,  // 101  75%
		0.90f,  // 110  90%
		0.50f   // 111  50% fallback
		};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define CA_DMA_TABLE_LEN 32u   // długość wzorca do DMA 16/64
static uint16_t tim2_ca_dma_table[CA_DMA_TABLE_LEN];  // wartości dla TIM2 CCR3
//ADC_HandleTypeDef hadc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
PwmBoxJumpers_t PwmBox_ReadJumpers(void);
// -- wyłączenie  zegara  HSI   dla  trybów bez ADC
void HSI_Enable(void);
void HSI_Disable(void);
// -- Ustawianie pinów  na analogowe  --
void Jumpers_SetModePinsToAnalog(void);
void Jumpers_SetFrequencyPinsToAnalog(void);
void Jumpers_SetPwmPinsToAnalog(void);
void Jumpers_SetEdgePinsToAnalog(void);
/* -- Wyłączenie jednego wyjścia i przygotowanie drugiego */
void RouteOutputToLPTIM(void);
void RouteOutputToTIM2(void);
// -- Trzy tryby pracy
void EnterRunMode(void);
void EnterSleepMode(void);
void EnterStopMode(void);

void Edge_SetOutputSlew(EdgeOutput_t output, EdgeSlew_t slew);

// --- ISENSE ADC helpery
static void ISense_ADC_Init(void);
static uint16_t ISense_ADC_ReadRaw(void);
static uint16_t ISense_MeasureSpan(uint16_t sample_count);

static float PwmBox_GetFrequencyHz(uint8_t idx);
static float PwmBox_GetDuty(uint8_t idx);
static void PwmBox_ConfigureTim2Pwm(float freq_hz, float duty);
static void PwmBox_ApplyMode(const PwmBoxJumpers_t *cfg);

static void Mode_LowEmiStop(const PwmBoxJumpers_t *cfg);
static void Mode_LowEmiSleep(const PwmBoxJumpers_t *cfg);
static void Mode_CaBase(const PwmBoxJumpers_t *cfg);
static void Mode_CaPrbs(const PwmBoxJumpers_t *cfg);
static void Mode_CaDma(const PwmBoxJumpers_t *cfg);
static void Mode_AutoTuneFreq(const PwmBoxJumpers_t *cfg);
static void Mode_AutoTuneSpread(const PwmBoxJumpers_t *cfg);
static void Mode_FixedTone(const PwmBoxJumpers_t *cfg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_LPTIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE BEGIN 2 */
	PwmBoxJumpers_t cfg = PwmBox_ReadJumpers();

	/* Ustaw tryb na podstawie zworek i wystartuj PWM */
	PwmBox_ApplyMode(&cfg);

	// Start PWM na TIM2, kanał 3
	// if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
	//{
	// Error_Handler();
	//}


	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);   // 50% duty
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
	PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HSI_Enable(void) {
	//Włącza HSI16
	__HAL_RCC_HSI_ENABLE();

	// stabilizacja
	while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {
	}
}

void HSI_Disable(void) {
	// Wyłącza HSI16
	__HAL_RCC_HSI_DISABLE();

	// stabilizacxja
	while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) {
	}
}

void Jumpers_SetModePinsToAnalog(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	//ręczne gpio dla zworek
	GPIO_InitStruct.Pin = TP_1_Pin | TP_2_Pin | TP_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Jumpers_SetFrequencyPinsToAnalog(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = Fset1_Pin | Fset2_Pin | Fset3_Pin | Fset4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Fset5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Jumpers_SetPwmPinsToAnalog(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = JPWM_1_Pin | JPWM_2_Pin | JPWM_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Jumpers_SetEdgePinsToAnalog(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = JEdge_2_Pin | JEdge_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = JEdge_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void RouteOutputToLPTIM(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Zatrzymuje PWM na TIM2_CH3 (PA2)
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

	//Ustaw PA2 (TIM2_CH3) na analog
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Ustawia PB2 jako wyjście LPTIM1_OUT
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_LPTIM1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void RouteOutputToTIM2(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Zatrzymuje PWM na LPTIM1_OUT (PB2)
	HAL_LPTIM_PWM_Stop(&hlptim1);

	// Ustawia PB2 (LPTIM1_OUT) na analog
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Ustawia PA2 jako wyjście TIM2_CH3
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void EnterRunMode(void) {
	SystemClock_Config();
}

void EnterSleepMode(void) {
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();
}

void EnterStopMode(void) {

	// Zatrzymaj SysTick
	HAL_SuspendTick();
	// Wejście w STOP z lowpower regulatorem,
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	/* Po wybudzeniu wracamy tutaj zegary są w stanie resetowym (MSI domyślnie),
	 * więc trzeba przywrócić konfigurację systemowego zegara.*/

	SystemClock_Config();
	// Wznów SysTick
	HAL_ResumeTick();
}
void Edge_SetOutputSlew(EdgeOutput_t output, EdgeSlew_t slew) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t alternate;


	switch (output) {
	case EDGE_OUTPUT_TIM2_CH3:
		port = GPIOA;
		pin = GPIO_PIN_2;
		alternate = GPIO_AF2_TIM2;
		break;
	case EDGE_OUTPUT_LPTIM1_OUT:
		port = GPIOB;
		pin = GPIO_PIN_2;
		alternate = GPIO_AF2_LPTIM1;
		break;
	default:
		return;
	}

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = alternate;

	switch (slew) {
	case EDGE_SLEW_LOW:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		break;
	case EDGE_SLEW_MEDIUM:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		break;
	case EDGE_SLEW_HIGH:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		break;
	case EDGE_SLEW_VERY_HIGH:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		break;
	default:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		break;
	}

	HAL_GPIO_Init(port, &GPIO_InitStruct);
}
PwmBoxJumpers_t PwmBox_ReadJumpers(void) {
	PwmBoxJumpers_t cfg;
	uint8_t b0, b1, b2, b3;
	uint8_t code;


	// 1. TRYB PRACY
	b0 = (HAL_GPIO_ReadPin(TP_3_GPIO_Port, TP_3_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	b1 = (HAL_GPIO_ReadPin(TP_2_GPIO_Port, TP_2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	b2 = (HAL_GPIO_ReadPin(TP_1_GPIO_Port, TP_1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	cfg.mode = (uint8_t) ((b2 << 2) | (b1 << 1) | b0);

	// 2, CZĘSTOTLIWOŚĆ freq_idx Fset1 na razie ignorujemy
	b0 = (HAL_GPIO_ReadPin(Fset5_GPIO_Port, Fset5_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	b1 = (HAL_GPIO_ReadPin(Fset4_GPIO_Port, Fset4_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	b2 = (HAL_GPIO_ReadPin(Fset3_GPIO_Port, Fset3_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	b3 = (HAL_GPIO_ReadPin(Fset2_GPIO_Port, Fset2_Pin) == GPIO_PIN_RESET) ?	1u : 0u;

	cfg.freq_idx = (uint8_t) ((b3 << 3) | (b2 << 2) | (b1 << 1) | b0);

	// 3. WYPEŁNIENIE duty_idx

	b0 = (HAL_GPIO_ReadPin(JPWM_1_GPIO_Port, JPWM_1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	b1 = (HAL_GPIO_ReadPin(JPWM_2_GPIO_Port, JPWM_2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	b2 = (HAL_GPIO_ReadPin(JPWM_3_GPIO_Port, JPWM_3_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
	cfg.duty_idx = (uint8_t) ((b2 << 2) | (b1 << 1) | b0);

	//  4, zbocza
	b0 = (HAL_GPIO_ReadPin(JEdge_1_GPIO_Port, JEdge_1_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	b1 = (HAL_GPIO_ReadPin(JEdge_2_GPIO_Port, JEdge_2_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	b2 = (HAL_GPIO_ReadPin(JEdge_3_GPIO_Port, JEdge_3_Pin) == GPIO_PIN_RESET) ?	1u : 0u;
	code = (uint8_t) ((b2 << 2) | (b1 << 1) | b0);

	switch (code) {
	case 0b000:
		cfg.edge_slew = EDGE_SLEW_LOW;
		break;
	case 0b100:
		cfg.edge_slew = EDGE_SLEW_MEDIUM;
		break;
	case 0b010:
		cfg.edge_slew = EDGE_SLEW_HIGH;
		break;
	case 0b001:
		cfg.edge_slew = EDGE_SLEW_VERY_HIGH;
		break;
	default:
		cfg.edge_slew = EDGE_SLEW_LOW; // konfiguracja inna niz wyzej  najbezpieczniej
		break;
	}

	// wszystkie wejscia na analog
	Jumpers_SetModePinsToAnalog();
	Jumpers_SetFrequencyPinsToAnalog();
	Jumpers_SetPwmPinsToAnalog();
	Jumpers_SetEdgePinsToAnalog();

	return cfg;
}

static float PwmBox_GetFrequencyHz(uint8_t idx) {
	if (idx >= (uint8_t) (sizeof(s_freq_table) / sizeof(s_freq_table[0]))) {
		idx = (uint8_t) (sizeof(s_freq_table) / sizeof(s_freq_table[0]) - 1u);
	}
	return s_freq_table[idx];
}

static float PwmBox_GetDuty(uint8_t idx) {
	if (idx >= (uint8_t) (sizeof(s_duty_table) / sizeof(s_duty_table[0]))) {
		idx = 3u; // bezpieczne 50proc
	}
	return s_duty_table[idx];
}

// Konfiguracja TIM2_CH3 f i D
static void PwmBox_ConfigureTim2Pwm(float freq_hz, float duty) {
	uint32_t tim_clk;
	RCC_ClkInitTypeDef clk;
	uint32_t flash_latency;

	// Częstotliwość zegara dla TIM2
	tim_clk = HAL_RCC_GetPCLK1Freq();
	HAL_RCC_GetClockConfig(&clk, &flash_latency);
	if (clk.APB1CLKDivider != RCC_HCLK_DIV1) {
		tim_clk *= 2u;   // klasyczna zasada- jeśli podzielony, timer = 2 * PCLK
	}

	if (freq_hz < 0.1f)
		freq_hz = 0.1f;
	if (duty < 0.0f)
		duty = 0.0f;
	if (duty > 0.99f)
		duty = 0.99f;

	double f_tim = (double) tim_clk;
	double f = (double) freq_hz;
	double ratio = f_tim / (f * 65536.0); // dobranie preskalera tak żeby ARR <= 0xFFFF
	uint32_t psc_reg;

	if (ratio < 1.0)
		psc_reg = 0u;
	else if (ratio > 65535.0)
		psc_reg = 65535u;
	else
		psc_reg = (uint32_t) ratio;

	double tmp = f_tim / (f * (double) (psc_reg + 1u)) - 1.0;
	if (tmp < 1.0)
		tmp = 1.0;
	if (tmp > 65535.0)
		tmp = 65535.0;

	uint32_t arr_reg = (uint32_t) (tmp + 0.5);    // zaokrąglenie

	__HAL_TIM_SET_PRESCALER(&htim2, (uint16_t )psc_reg);
	__HAL_TIM_SET_AUTORELOAD(&htim2, (uint16_t )arr_reg);

	//CCR z wypełnienia
	uint32_t ccr = (uint32_t) ((double) (arr_reg + 1u) * (double) duty + 0.5);
	if (ccr > arr_reg)
		ccr = arr_reg;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t )ccr);

	// Zresetuj licznik po zmianie parametrów
	__HAL_TIM_SET_COUNTER(&htim2, 0u);
}

// Główna funkcja wybiera tryb na podstawie zworki i uruchamia PWM
static void PwmBox_ApplyMode(const PwmBoxJumpers_t *cfg) {
	switch (cfg->mode) {
	case PWMBOX_MODE_LOWEMI_STOP:
		Mode_LowEmiStop(cfg);
		break;

	case PWMBOX_MODE_LOWEMI_SLEEP:
		Mode_LowEmiSleep(cfg);
		break;

	case PWMBOX_MODE_CA_BASE:
		Mode_CaBase(cfg);
		break;

	case PWMBOX_MODE_CA_PRBS:
		Mode_CaPrbs(cfg);
		break;

	case PWMBOX_MODE_CA_DMA:
		Mode_CaDma(cfg);
		break;

	case PWMBOX_MODE_AUTOTUNE_FREQ:
		Mode_AutoTuneFreq(cfg);
		break;

	case PWMBOX_MODE_AUTOTUNE_SPREAD:
		Mode_AutoTuneSpread(cfg);
		break;

	case PWMBOX_MODE_FIXED_TONE:
		Mode_FixedTone(cfg);
		break;

	default:
		// Fallback -  nie  ma
		Mode_FixedTone(cfg);
		break;
	}

	// Zworki już są odczytane – linie można wyciszyć.

	Jumpers_SetModePinsToAnalog();
	Jumpers_SetFrequencyPinsToAnalog();
	Jumpers_SetPwmPinsToAnalog();
	Jumpers_SetEdgePinsToAnalog();
}

// --- Tryb 000: LowEMI-STOP ----------------------------------------
static void Mode_LowEmiStop(const PwmBoxJumpers_t *cfg) {
	// Częstotliwość i wypełnienie wynikające ze zworek
	float freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);

	/* 1. Dobór preskalera i okresu dla LPTIM1 taktowanego z LSI
	 *    LPTIM jest 16-bitowy, więc ARR <= 0xFFFF.
	 *
	 */
	uint32_t lptim_clk = LSI_VALUE;  // wartość z biblioteki system

	static const uint32_t presc_values[8] = { 1u, 2u, 4u, 8u, 16u, 32u, 64u,
			128u };

	static const uint32_t presc_cfg[8] = {
	LPTIM_PRESCALER_DIV1,
	LPTIM_PRESCALER_DIV2,
	LPTIM_PRESCALER_DIV4,
	LPTIM_PRESCALER_DIV8,
	LPTIM_PRESCALER_DIV16,
	LPTIM_PRESCALER_DIV32,
	LPTIM_PRESCALER_DIV64,
	LPTIM_PRESCALER_DIV128 };

	uint32_t period = 0xFFFFu;
	uint32_t presc_bits = LPTIM_PRESCALER_DIV128;   // domyślnie najwolniejszy

	for (int i = 0; i < 8; ++i) {
		float ticks_f = (float) lptim_clk / (freq_hz * (float) presc_values[i]);

		if (ticks_f >= 1.0f && ticks_f <= 65536.0f) {
			// Zaokrąglenie do najbliższej liczby całkowitej
			uint32_t ticks = (uint32_t) (ticks_f + 0.5f);
			if (ticks == 0u) {
				ticks = 1u;
			}

			period = ticks - 1u;     // ARR = ticks - 1
			presc_bits = presc_cfg[i];  // preskaler dla LPTIM
			break;
		}
	}

	// 2. Obliczenie wypełnienia
	uint32_t pulse = (uint32_t) ((float) (period + 1u) * duty + 0.5f);
	if (pulse > period) {
		pulse = period;
	}

	/* 3. Przełączenie wyjścia na LPTIM1_OUT PB2,
	 *    wyłączenie TIM2_CH3  robi RouteOutputToLPTIM.
	 */
	RouteOutputToLPTIM();
	Edge_SetOutputSlew(EDGE_OUTPUT_LPTIM1_OUT, cfg->edge_slew);
	HSI_Disable();
	__HAL_LPTIM_DISABLE(&hlptim1);
	hlptim1.Init.Clock.Prescaler = presc_bits;
	if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_LPTIM_PWM_Start(&hlptim1, period, pulse) != HAL_OK) {
		Error_Handler();
	}
	EnterStopMode();
}

// --- Tryb 001 LowEMI-SLEEP ----------
static void Mode_LowEmiSleep(const PwmBoxJumpers_t *cfg) {
	/* Częstotliwość i wypełnienie ze zworek */
	float freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);

	if (freq_hz <= 0.0f) {
		freq_hz = 1.0f;
	}

	/* 1. Dobór preskalera i okresu dla LPTIM1
	 *    - LPTIM1 16-bit  ARR <= 0xFFFF
	 *    - LSI_VALUE jako f_clk
	 */
	uint32_t lptim_clk = LSI_VALUE;  // z system

	static const uint32_t presc_values[8] = { 1u, 2u, 4u, 8u, 16u, 32u, 64u,
			128u };

	static const uint32_t presc_cfg[8] = {
	LPTIM_PRESCALER_DIV1,
	LPTIM_PRESCALER_DIV2,
	LPTIM_PRESCALER_DIV4,
	LPTIM_PRESCALER_DIV8,
	LPTIM_PRESCALER_DIV16,
	LPTIM_PRESCALER_DIV32,
	LPTIM_PRESCALER_DIV64,
	LPTIM_PRESCALER_DIV128 };

	uint32_t period = 0xFFFFu;
	uint32_t presc_bits = LPTIM_PRESCALER_DIV128;   // fallback – najwolniejszy

	for (int i = 0; i < 8; ++i) {
		float ticks_f = (float) lptim_clk / (freq_hz * (float) presc_values[i]);

		if (ticks_f >= 1.0f && ticks_f <= 65536.0f) {
			uint32_t ticks = (uint32_t) (ticks_f + 0.5f);
			if (ticks == 0u) {
				ticks = 1u;
			}

			period = ticks - 1u;      // ARR
			presc_bits = presc_cfg[i];    // preskaler
			break;
		}
	}

	/* 2. Wypełnienie */
	uint32_t pulse = (uint32_t) ((float) (period + 1u) * duty + 0.5f);
	if (pulse > period) {
		pulse = period;
	}

	/* 3. Przełączenie wyjścia na LPTIM1_OUT  wyciszenie TIM2_CH3  */
	RouteOutputToLPTIM();

	Edge_SetOutputSlew(EDGE_OUTPUT_LPTIM1_OUT, cfg->edge_slew);

	HSI_Disable();

	/* 5. Ustawienie preskalera i start LPTIM1 PWM */
	__HAL_LPTIM_DISABLE(&hlptim1);
	hlptim1.Init.Clock.Prescaler = presc_bits;
	if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_LPTIM_PWM_Start(&hlptim1, period, pulse) != HAL_OK) {
		Error_Handler();
	}

	/* 6. Wejście w SLEEP – CPU śpi, LPTIM1 + LSI dalej robi PWM */
	EnterSleepMode();

}

/* --- Tryb 010: CA-Base -------------------------------------------- */

static void Mode_CaBase(const PwmBoxJumpers_t *cfg) {
	/* 1. Wylicz częstotliwość i wypełnienie z indeksów zworek */
	float freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);

	EnterRunMode();

	RouteOutputToTIM2();

	Edge_SetOutputSlew(EDGE_OUTPUT_TIM2_CH3, cfg->edge_slew);

	PwmBox_ConfigureTim2Pwm(freq_hz, duty);

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

}

/* --- Tryb 011: CA-PRBS -------------------------------------------- */

static void Mode_CaPrbs(const PwmBoxJumpers_t *cfg) {
	/* 1. Bazowa częstotliwość i wypełnienie ze zworek */
	float freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);

	Jumpers_SetModePinsToAnalog();
	Jumpers_SetFrequencyPinsToAnalog();
	Jumpers_SetPwmPinsToAnalog();
	Jumpers_SetEdgePinsToAnalog();

	EnterRunMode();

	RouteOutputToTIM2();

	Edge_SetOutputSlew(EDGE_OUTPUT_TIM2_CH3, cfg->edge_slew);

	PwmBox_ConfigureTim2Pwm(freq_hz, duty);

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

	uint16_t base_period = (uint16_t) __HAL_TIM_GET_AUTORELOAD(&htim2);
	uint16_t base_pulse = (uint16_t) __HAL_TIM_GET_COMPARE(&htim2,
			TIM_CHANNEL_3);

	if (base_period < 10u) {
		base_period = 10u;
	}
	if (base_pulse == 0u) {
		base_pulse = 1u;
	}
	if (base_pulse >= base_period) {
		base_pulse = (uint16_t) (base_period - 1u);
	}

	//  Parametry ditheru
	uint16_t max_delta = (uint16_t) (base_period / 50u);  // ~2%
	if (max_delta < 1u) {
		max_delta = 1u;
	}

	/*  16-bitowy LFSR PRBS  polinom x^16 + x^14 + x^13 + x^11 + 1 */
	uint16_t lfsr = 0xACE1u;   // dowolny niezerowy seed

	const uint32_t prbs_every_cycles = 10u;
	uint32_t cycle_counter = 0u;

	/* 10. Główna pętla: aktualizacja PRBS NA GRANICY OKRESU TIM2 */
	while (1) {
		/*  Czekamy na UPDATE timera koniec okresu */
		while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET) {

		}
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

		/*  Liczymy okresy – PRBS co N okresów */
		cycle_counter++;
		if (cycle_counter < prbs_every_cycles) {
			/* Ten okres jedzie jeszcze na starych ustawieniach */
			continue;
		}
		cycle_counter = 0u;

		/*  Krok LFSR  */
		uint16_t lsb = lfsr & 1u;
		lfsr >>= 1;
		if (lsb) {
			lfsr ^= 0xB400u;
		}

		/*  PRBS  */
		int16_t prbs_raw = (int16_t) (lfsr & 0x00FFu);  // 0-255
		prbs_raw -= 128;                               // -128-127

		int32_t delta = ((int32_t) prbs_raw * (int32_t) max_delta) / 128; // ±max_delta

		/*  Nowy okres wokół base_period, w bezpiecznym zakresie */
		int32_t new_period32 = (int32_t) base_period + delta;
		if (new_period32 < 10) {
			new_period32 = 10;
		}
		if (new_period32 > 65535) {
			new_period32 = 65535;
		}
		uint16_t new_period = (uint16_t) new_period32;

		/*  Przeskaluj impuls, żeby duty stałe (ok. 50%) */
		uint32_t new_pulse32 = (uint32_t) new_period * (uint32_t) base_pulse
				/ (uint32_t) base_period;

		if (new_pulse32 == 0u) {
			new_pulse32 = 1u;
		}
		if (new_pulse32 >= new_period) {
			new_pulse32 = (uint32_t) new_period - 1u;
		}
		uint16_t new_pulse = (uint16_t) new_pulse32;

		/*  Zastosuj nowe ARR/CCR – NA GRANICY OKRESU
		 *      (właśnie dlatego czekamy na TIM_FLAG_UPDATE).
		 */
		__HAL_TIM_SET_AUTORELOAD(&htim2, new_period);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, new_pulse);
	}
}

/* --- Tryb 100: CA-DMA --------------------------------------------- */

static void Mode_CaDma(const PwmBoxJumpers_t *cfg) {
	/* 1. Bazowa częstotliwość i wypełnienie */
	float freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);

	if (freq_hz <= 0.0f) {
		freq_hz = 1.0f;
	}
	if (duty < 0.0f) {
		duty = 0.0f;
	}
	if (duty > 0.99f) {
		duty = 0.99f;
	}


	Jumpers_SetModePinsToAnalog();
	Jumpers_SetFrequencyPinsToAnalog();
	Jumpers_SetPwmPinsToAnalog();
	Jumpers_SetEdgePinsToAnalog();

	EnterRunMode();

	RouteOutputToTIM2();

	Edge_SetOutputSlew(EDGE_OUTPUT_TIM2_CH3, cfg->edge_slew);

	/* Bazowy PWM */
	PwmBox_ConfigureTim2Pwm(freq_hz, duty);

	/*  Odczyt ARR i CCR jako środek „oddechu” */
	uint16_t base_period = (uint16_t) __HAL_TIM_GET_AUTORELOAD(&htim2);
	uint16_t base_pulse = (uint16_t) __HAL_TIM_GET_COMPARE(&htim2,
			TIM_CHANNEL_3);

	if (base_period < 10u) {
		base_period = 10u;
	}
	if (base_pulse == 0u) {
		base_pulse = 1u;
	}
	if (base_pulse >= base_period) {
		base_pulse = (uint16_t) (base_period - 1u);
	}

	// 8. Głębokość oddechu
	int32_t amp = (int32_t) (base_period / 50u);
	if (amp < 1) {
		amp = 1;
	}

	// 9. Wypełnij tablicę tim2_ca_dma_table[] trójkątnym wzorcem

	const uint32_t half_len = CA_DMA_TABLE_LEN / 2u;
	uint32_t i;

	for (i = 0u; i < CA_DMA_TABLE_LEN; i++) {
		int32_t x;

		if (i < half_len) {
			//rampa góra
			int32_t num = (int32_t) (2 * amp) * (int32_t) i;
			int32_t den = (int32_t) ((half_len > 1u) ? (half_len - 1u) : 1u);
			int32_t delta = (den != 0) ? (num / den) : 0;
			x = (int32_t) base_pulse - amp + delta;
		} else {
			//rampa dół
			uint32_t j = i - half_len;
			int32_t num = (int32_t) (2 * amp) * (int32_t) j;
			int32_t den = (int32_t) ((half_len > 1u) ? (half_len - 1u) : 1u);
			int32_t delta = (den != 0) ? (num / den) : 0;
			x = (int32_t) base_pulse + amp - delta;
		}
		/* Zabezpieczenia: brak 0% i 100% duty mruganie  */
		if (x < 1) {
			x = 1;
		}
		if (x >= (int32_t) base_period) {
			x = (int32_t) base_period - 1;
		}

		tim2_ca_dma_table[i] = (uint16_t) x;
	}

	if (HAL_TIM_PWM_Start_DMA(&htim2,
	TIM_CHANNEL_3, (uint32_t*) tim2_ca_dma_table,
	CA_DMA_TABLE_LEN) != HAL_OK) {
		Error_Handler();
	}
	EnterSleepMode();
	while (1) {
	}
}

/* --- Tryb 101: AutoTune-Freq -------------------------------------- */

static void Mode_AutoTuneFreq(const PwmBoxJumpers_t *cfg) {
	float base_freq_hz = PwmBox_GetFrequencyHz(cfg->freq_idx);
	float duty = PwmBox_GetDuty(cfg->duty_idx);


	/* Wyjście: TIM2_CH3 na PA2 */
	RouteOutputToTIM2();
	Edge_SetOutputSlew(EDGE_OUTPUT_TIM2_CH3, cfg->edge_slew);

	/* HSI potrzebne do zegara ADC1 */
	HSI_Enable();

	/* Inicjalizacja ADC dla ISENSE (PA0 / ADC_CHANNEL_0) */
	ISense_ADC_Init();

	/* Start z częstotliwością bazową */
	PwmBox_ConfigureTim2Pwm(base_freq_hz, duty);
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

		if (base_freq_hz < 10.0f) {// granica skanowania
		return;
	}


	static const float rel_offsets[] = { -0.10f, -0.05f, 0.0f, 0.05f, 0.10f };
	const uint32_t sample_count = 64u;

	float best_freq_hz = base_freq_hz;
	uint32_t best_metric = 0xFFFFFFFFu;

	for (uint32_t i = 0; i < (sizeof(rel_offsets) / sizeof(rel_offsets[0]));
			++i) {
		float test_freq = base_freq_hz * (1.0f + rel_offsets[i]);

		/* Nie schodzimy poniżej progu, żeby nie czekać wieki na ustalenie się układu */
		if (test_freq < 10.0f) {
			test_freq = 10.0f;
		}

		/* Przestrojenie TIM2 na nową częstotliwość przy tym samym wypełnieniu */
		PwmBox_ConfigureTim2Pwm(test_freq, duty);

		/* Krótka pauza, żeby filtr/obciążenie złapały nowy stan.
		 * Przy f rzędu setek Hz 5–10 ms to kilka-kilkanaście okresów. */
		HAL_Delay(10);

		/* Prosta metryka „szumu”: rozpiętość max–min na ISENSE */
		uint32_t span = ISense_MeasureSpan(sample_count);

		if (span < best_metric) {
			best_metric = span;
			best_freq_hz = test_freq;
		}
	}

	PwmBox_ConfigureTim2Pwm(best_freq_hz, duty);


	HAL_ADC_DeInit(&hadc);
	__HAL_RCC_ADC1_CLK_DISABLE();
	HSI_Disable();

}

/* --- Tryb 110: AutoTune-Spread ------------------------------------ */
static void Mode_AutoTuneSpread(const PwmBoxJumpers_t *cfg) {
	//
}

/* --- Tryb 111: FixedTone ------------------------------------------ */
static void Mode_FixedTone(const PwmBoxJumpers_t *cfg) {
	//
}


// Prosta inicjalizacja ADC1
static void ISense_ADC_Init(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* Ustaw instancję */
	hadc.Instance = ISENSE_ADC_INSTANCE;

	/* Konfiguracja zegara i podstawowych parametrów ADC (STM32L0) */
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;          // L0 bez DIV1
	hadc.Init.Resolution = ADC_RESOLUTION12b;        // 12 bitów
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;              // pojedyncza konwersja
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = OVR_DATA_PRESERVED;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;

	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ISENSE_ADC_CHANNEL;        // ADC_CHANNEL_0
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static uint16_t ISense_MeasureSpan(uint16_t sample_count) {
	if (sample_count == 0) {
		return 0;
	}

	uint16_t min = 0xFFFF;
	uint16_t max = 0x0000;

	for (uint16_t i = 0; i < sample_count; ++i) {
		uint16_t v = ISense_ADC_ReadRaw();

		if (v < min)
			min = v;
		if (v > max)
			max = v;
	}

	return (uint16_t) (max - min);
}

// Jednorazowy odczyt z ISENSE
static uint16_t ISense_ADC_ReadRaw(void) {
	uint32_t value = 0;

	if (HAL_ADC_Start(&hadc) != HAL_OK) {
		return 0;
	}

	// maks. 10 ms na konwersję
	if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK) {
		HAL_ADC_Stop(&hadc);
		return 0;
	}

	value = HAL_ADC_GetValue(&hadc);

	HAL_ADC_Stop(&hadc);

	return (uint16_t) value;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
