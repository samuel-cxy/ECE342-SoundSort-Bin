/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "VL53L1X_api.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Total DMA buffer size for I2S audio
// DMA will fill this buffer continuously in a ping-pong manner:
// first half -> interrupt, second half -> interrupt
#define I2S_DMA_SAMPLES 1024
#define HALF_SAMPLES (I2S_DMA_SAMPLES / 2)

// If the ToF sensor sees an object closer than this, start listening
#define TOF_THRESHOLD_MM 150

// We store 0.5 seconds at 16kHz audio
// 16000 samples/sec * 0.5 sec = 8000 samples
#define AUDIO_LEN 8000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float PI = 3.14159265358979f;

// Raw DMA buffer filled directly by I2S peripheral
static uint16_t i2s_rx_buf[I2S_DMA_SAMPLES];

// Final mono audio clip used for feature extraction
// This holds 8000 float samples = 0.5 seconds
static float audio_buf[AUDIO_LEN]; // Store 0.5s of audio as floats

// Current write index into audio_buf[]
static uint32_t audio_idx = 0;

// Set to 1 once we detect the actual impact sound
// Before that, we keep waiting even though recording has started
static uint8_t is_impact_detected = 0;

// Flags set by the DMA hardware interrupts
volatile uint8_t half_ready = 0;
volatile uint8_t full_ready = 0;

// Running DC estimate for a simple DC blocker
// Helps remove microphone bias / offset
static int32_t dc_offset = 0;

// FSM States
typedef enum
{
	STATE_IDLE,		// Waiting for an object to fall
	STATE_RECORDING // Object detected, currently filling audio_buf
} SystemState;

SystemState current_state = STATE_IDLE;
uint32_t chunk_count = 0;
uint16_t dev = 0x52; // ToF I2C Address

// Parses the raw hardware I2S buffer and converts it to a standard float array
static void fill_audio_buffer(uint16_t *src)
{
	for (uint32_t i = 0; i < HALF_SAMPLES; i += 4)
	{
		int16_t raw_sample = (int16_t)src[i];
		dc_offset = dc_offset + ((raw_sample - dc_offset) >> 6);

		// RAW, clean conversion. No 8x multiplier, no clipping.
		float sample_float = (float)(raw_sample - dc_offset) / 32768.0f;

		if (chunk_count < 5)
			continue; // Skip the startup pop

		float abs_val = fabsf(sample_float);

		// Wait for impact
		if (!is_impact_detected)
		{
			if (abs_val > 0.015f)
			{ // Standard trigger threshold
				is_impact_detected = 1;
			}
		}

		// Record exactly 0.5 seconds
		if (is_impact_detected && audio_idx < AUDIO_LEN)
		{
			audio_buf[audio_idx++] = sample_float;
		}
	}
}

// DMA Call Backs
// Ping-Pong buffering: while the CPU processes the first half, the DMA fills the second half
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (hi2s->Instance == SPI2)
	{
		half_ready = 1;
	}
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (hi2s->Instance == SPI2)
	{
		full_ready = 1;
	}
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
	MX_DMA_Init();
	MX_I2S2_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	// 1. Turn on RED LED to indicate boot is starting
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);

	// 2. Force the ToF Sensor OFF (Pull XSHUT Low)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(10); // Hold it in reset

	// 3. Wake the ToF Sensor UP (Pull XSHUT High)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(50); // Give it time to boot internally

	uint8_t sensorState = 0;
	while (sensorState == 0)
	{
		VL53L1X_BootState(dev, &sensorState);
		HAL_Delay(2);
	}

	// 4. Initialize ToF Sensor for fast, short-range detection
	VL53L1X_SensorInit(dev);
	VL53L1X_SetDistanceMode(dev, 1);
	VL53L1X_SetTimingBudgetInMs(dev, 33);
	VL53L1X_SetInterMeasurementInMs(dev, 33);
	VL53L1X_StartRanging(dev);

	// 5. Success! Turn OFF Red LED, Turn ON Green LED
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (current_state == STATE_IDLE)
		{
			// Poll the Time-of-Flight sensor to see if it finished a measurement
			uint8_t dataReady = 0;
			VL53L1X_CheckForDataReady(dev, &dataReady);

			if (dataReady)
			{
				uint16_t distance = 0;
				VL53L1X_GetDistance(dev, &distance);
				VL53L1X_ClearInterrupt(dev);

				// TRIGGER DETECTED!
				if (distance < TOF_THRESHOLD_MM && distance > 0)
				{
					current_state = STATE_RECORDING;
					chunk_count = 0;
					audio_idx = 0; // Reset our main float buffer index
					half_ready = 0;
					full_ready = 0;

					// Stop ToF Ranging so the I2C bus doesn't interrupt the audio processing
					VL53L1X_StopRanging(dev);

					// Tell the hardware DMA to start dumping microphone data into our array
					HAL_I2S_Receive_DMA(&hi2s2, i2s_rx_buf, I2S_DMA_SAMPLES);

					char msg[64];
					sprintf(msg, "--- ITEM DETECTED (%d mm) ---\n", distance);
					HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg),
									  HAL_MAX_DELAY);
				}
			}
		}
		else if (current_state == STATE_RECORDING)
		{
			// The DMA finished filling the first half of the array
			if (half_ready)
			{
				half_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[0]);
				chunk_count++;
			}

			// The DMA finished filling the second half of the array
			if (full_ready)
			{
				full_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[HALF_SAMPLES]);
				chunk_count++;
			}

			// ---------------------------------------------------------
			// 1. SUCCESS: Did we finish collecting exactly 0.5s of audio?
			// ---------------------------------------------------------
			if (audio_idx >= AUDIO_LEN)
			{
				HAL_I2S_DMAStop(&hi2s2);

				// 1. Send Start Marker
				char start_msg[] = "START_AUDIO\n";
				HAL_UART_Transmit(&huart3, (uint8_t *)start_msg,
								  strlen(start_msg), HAL_MAX_DELAY);

				// 2. Transmit each sample as a human-readable text string!
				// This completely immunizes the data from binary alignment corruption.
				for (int i = 0; i < AUDIO_LEN; i++)
				{
					int16_t pcm_val = (int16_t)(audio_buf[i] * 32767.0f);
					char val_str[16];

					// Convert the integer to text with a newline
					sprintf(val_str, "%d\n", pcm_val);
					HAL_UART_Transmit(&huart3, (uint8_t *)val_str,
									  strlen(val_str), HAL_MAX_DELAY);
				}

				// 3. Send End Marker
				char end_msg[] = "END_AUDIO\n";
				HAL_UART_Transmit(&huart3, (uint8_t *)end_msg, strlen(end_msg),
								  HAL_MAX_DELAY);

				// Wait 1 second before allowing the next drop
				HAL_Delay(1000);

				VL53L1X_StartRanging(dev);
				is_impact_detected = 0;
				current_state = STATE_IDLE;
			}
			// ---------------------------------------------------------
			// 2. TIMEOUT: Did we wait 2 seconds and NEVER hear an impact?
			// ---------------------------------------------------------
			else if (chunk_count >= 250 && !is_impact_detected)
			{
				// Stop the microphone!
				HAL_I2S_DMAStop(&hi2s2);

				char msg[] = "TIMEOUT: Object detected, but no sound heard.\n\n";
				HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg),
								  HAL_MAX_DELAY);

				// Only wait half a second on a timeout before scanning again
				HAL_Delay(500);
				VL53L1X_StartRanging(dev);
				is_impact_detected = 0;
				current_state = STATE_IDLE;
			}
		}
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
	hi2c1.Init.ClockSpeed = 400000;
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
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */
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
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
					  GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
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
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
