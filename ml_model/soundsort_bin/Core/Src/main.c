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
#include "soundsort_model.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// DMA buffer size for I2S audio.
// 1024 samples = 512 for the first half, 512 for the second half.
#define I2S_DMA_SAMPLES 1024
#define HALF_SAMPLES (I2S_DMA_SAMPLES / 2)

#define TOF_THRESHOLD_MM 100
#define TARGET_SR 16000

#define AUDIO_LEN 8000 // 0.5 seconds at 16kHz

// Calculate how many I2S DMA interrupts it takes to fill our 0.5s buffer.
// Divide HALF_SAMPLES by 4 because the STM32 I2S outputs 64-bit frames
// (4 x 16-bit slots per frame) for stereo, but only want one mono channel.
#define CHUNKS_NEEDED (AUDIO_LEN / (HALF_SAMPLES / 4)) // Roughly 63 chunks
#define N_FFT 1024
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

static uint16_t i2s_rx_buf[I2S_DMA_SAMPLES];
static float audio_buf[AUDIO_LEN]; // Store 0.5s of audio as floats
static uint32_t audio_idx = 0;

// Prevent Stack Overflow for extract_features()
static float fft_in[1024];
static float P[513];
static uint8_t is_impact_detected = 0; // flag for impact trigger
static float max_heard_volume = 0.0f;
static int16_t pcm_tx[AUDIO_LEN];

// Flags set by the DMA hardware interrupts
volatile uint8_t half_ready = 0;
volatile uint8_t full_ready = 0;

static int32_t dc_offset = 0;

// FSM States
typedef enum {
	STATE_IDLE,     // Waiting for an object to fall
	STATE_RECORDING // Object detected, currently filling audio_buf
} SystemState;

SystemState current_state = STATE_IDLE;
uint32_t chunk_count = 0;
uint16_t dev = 0x52; // ToF I2C Address

// Computes a Fast Fourier Transform (FFT) using the Cooley-Tukey Radix-2 algorithm
void compute_power_spectrum(float *in, float *P) {
	int N = 1024;
	float real[1024];
	float imag[1024];

	// 1. Copy the input audio into the complex arrays
	for (int i = 0; i < N; i++) {
		real[i] = in[i];
		imag[i] = 0.0f;
	}

	// 2. Bit-reversal sorting (reorders the data for the FFT)
	int j = 0;
	for (int i = 0; i < N - 1; i++) {
		if (i < j) {
			float tr = real[j];
			float ti = imag[j];
			real[j] = real[i];
			imag[j] = imag[i];
			real[i] = tr;
			imag[i] = ti;
		}
		int k = N / 2;
		while (k <= j) {
			j -= k;
			k /= 2;
		}
		j += k;
	}

	// 3. The Core FFT Algorithm (O(N log N) complexity)
	for (int l = 1; l <= 10; l++) { // 2^10 = 1024 bins
		int m = 1 << l;
		int m2 = m / 2;

		// We only call sin/cos 10 times total now!
		float omega_real = cosf(-2.0f * PI / m);
		float omega_imag = sinf(-2.0f * PI / m);

		for (int k = 0; k < N; k += m) {
			float w_real = 1.0f;
			float w_imag = 0.0f;
			for (int i = 0; i < m2; i++) {
				int t_idx = k + i + m2;
				float tr = w_real * real[t_idx] - w_imag * imag[t_idx];
				float ti = w_real * imag[t_idx] + w_imag * real[t_idx];

				int u_idx = k + i;
				real[t_idx] = real[u_idx] - tr;
				imag[t_idx] = imag[u_idx] - ti;
				real[u_idx] += tr;
				imag[u_idx] += ti;

				// Advance the angle
				float next_w_real = w_real * omega_real - w_imag * omega_imag;
				w_imag = w_real * omega_imag + w_imag * omega_real;
				w_real = next_w_real;
			}
		}
	}

	// 4. Calculate the final power spectrum for the first 513 bins
	for (int k = 0; k <= 512; k++) {
		P[k] = real[k] * real[k] + imag[k] * imag[k];
	}
}

// Replicates the Python extract_features() function entirely in C
void extract_features(float *x, float *feats) {
	// ==========================================
	// THE FIX: "Trim Silence" Zero-Padding
	// ==========================================
	// 1. Find the last sample that is louder than the 0.03f threshold
	int last_active_idx = 0;
	for (int i = 0; i < AUDIO_LEN; i++) {
		if (fabsf(x[i]) > 0.03f) {
			last_active_idx = i;
		}
	}

	// 2. Overwrite everything after the impact finishes with pure zeros.
	// This perfectly matches the np.zeros padding from your Python training!
	for (int i = last_active_idx + 1; i < AUDIO_LEN; i++) {
		x[i] = 0.0f;
	}

	// ==========================================
	// 1. Time-Domain Features
	// ==========================================
	float sum_sq = 0, peak = 0, zcr = 0;
	int active_count = 0;

	// Sweep through the 0.5 second audio buffer once
	for (int i = 0; i < AUDIO_LEN; i++) {
		float val = x[i];
		float abs_val = fabsf(val);

		sum_sq += val * val; // Accumulate for RMS
		if (abs_val > peak)
			peak = abs_val; // Find the loudest peak
		if (abs_val > 0.05f)
			active_count++; // Count "loud" samples for active ratio

		// Count how many times the audio waveform crosses the zero line
		if (i > 0) {
			if ((x[i] >= 0 && x[i - 1] < 0) || (x[i] < 0 && x[i - 1] >= 0))
				zcr += 1.0f;
		}
	}

	// Add 1e-12f to prevent division-by-zero errors in case of pure silence
	feats[0] = sqrtf(sum_sq / AUDIO_LEN + 1e-12f); // Root Mean Square (Loudness)
	feats[1] = peak + 1e-12f;                      // Peak Amplitude
	feats[2] = zcr / (AUDIO_LEN - 1);              // Zero Crossing Rate

	// --- Decay Ratio ---
	// Compares the energy of the first 1/3rd of the clip to the last 1/3rd
	// Glass usually rings (long decay), while plastic has a dull thud (fast decay)
	int third = AUDIO_LEN / 3;
	float e1 = 1e-12f, e3 = 1e-12f;
	for (int i = 0; i < third; i++)
		e1 += x[i] * x[i];
	for (int i = AUDIO_LEN - third; i < AUDIO_LEN; i++)
		e3 += x[i] * x[i];
	feats[3] = logf(e1 / e3);

	// ==========================================
	// 2. Frequency-Domain Features
	// ==========================================
	for (int i = 0; i < 1024; i++) {
		// Apply a Hanning window. This smooths the edges of the audio chunk
		// to zero so the math doesn't create artificial high-frequency noise.
		float window = 0.5f * (1.0f - cosf(2.0f * PI * i / (AUDIO_LEN - 1)));
		fft_in[i] = x[i] * window;
	}

	// Get the power spectrum
	compute_power_spectrum(fft_in, P);

	float total_energy = 1e-12f;
	for (int k = 0; k <= 512; k++)
		total_energy += P[k];

	// --- Band Energies & Spectral Centroid ---
	// Each "bin" in our FFT represents ~15.625 Hz (16000 SR / 1024 size)
	float bin_width = TARGET_SR / 1024.0f;
	float centroid_sum = 0;
	int edges[] = { 0, 300, 700, 1200, 2000, 3000, 4500, 6500, 8000 };

	for (int b = 0; b < 8; b++) {
		float band_e = 0;
		for (int k = 0; k <= 512; k++) {
			float freq = k * bin_width;

			// If the frequency of this bin falls inside our current band edge, sum it
			if (freq >= edges[b] && freq < edges[b + 1])
				band_e += P[k];

			// Only calculate the centroid (weighted average of frequencies) during the first pass
			if (b == 0)
				centroid_sum += freq * P[k];
		}
		// Normalize the band energy against the total energy
		feats[7 + b] = band_e / total_energy;
	}

	feats[4] = centroid_sum / total_energy; // Spectral Centroid

	// --- Spectral Rolloff ---
	// Find the frequency below which 85% of the audio's energy exists
	float cumsum = 0;
	float rolloff_freq = 0;
	for (int k = 0; k <= 512; k++) {
		cumsum += P[k];
		if (cumsum >= 0.85f * total_energy) {
			rolloff_freq = k * bin_width;
			break;
		}
	}
	feats[5] = rolloff_freq;
	feats[6] = (float) active_count / AUDIO_LEN; // Active Ratio
}

// Runs the logistic regression inference using your exported weights
// Returns the integer index of the winning class (0=can, 1=glass, etc.)
int classify_garbage(float *feats) {
	int best_class = 0;
	float max_score = -1e9f; // Start with a very low score

	// For each possible garbage type...
	for (int c = 0; c < NUM_CLASSES; c++) {
		float z = LR_BIAS[c]; // Start with the bias term

		// ...multiply each extracted feature by its corresponding weight
		for (int f = 0; f < NUM_FEATURES; f++) {
			// Standardize the feature using the Scikit-Learn scaler data first
			float norm_val = (feats[f] - FEATURE_MEAN[f]) / FEATURE_SCALE[f];
			z += norm_val * LR_WEIGHTS[c][f];
		}

		// Keep track of whichever class scores the highest
		if (z > max_score) {
			max_score = z;
			best_class = c;
		}
	}
	return best_class;
}

// Parses the raw hardware I2S buffer and converts it to a standard float array
static void fill_audio_buffer(uint16_t *src) {
	for (uint32_t i = 0; i < HALF_SAMPLES; i += 4) {
		int16_t raw_sample = (int16_t) src[i];
		dc_offset = dc_offset + ((raw_sample - dc_offset) >> 6);

		// RAW, clean conversion. No 8x multiplier, no clipping.
		float sample_float = (float) (raw_sample - dc_offset) / 32768.0f;

		if (chunk_count < 5)
			continue; // Skip the startup pop

		float abs_val = fabsf(sample_float);

		// Wait for impact
		if (!is_impact_detected) {
			if (abs_val > 0.015f) { // Standard trigger threshold
				is_impact_detected = 1;
			}
		}

		// Record exactly 0.5 seconds
		if (is_impact_detected && audio_idx < AUDIO_LEN) {
			audio_buf[audio_idx++] = sample_float;
		}
	}
}

// DMA Call Backs
// Ping-Pong buffering: while the CPU processes the first half, the DMA fills the second half
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == SPI2) {
		half_ready = 1;
	}
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == SPI2) {
		full_ready = 1;
	}
}
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
	while (sensorState == 0) {
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
	while (1) {
		if (current_state == STATE_IDLE) {
			// Poll the Time-of-Flight sensor to see if it finished a measurement
			uint8_t dataReady = 0;
			VL53L1X_CheckForDataReady(dev, &dataReady);

			if (dataReady) {
				uint16_t distance = 0;
				VL53L1X_GetDistance(dev, &distance);
				VL53L1X_ClearInterrupt(dev);

				// TRIGGER DETECTED!
				if (distance < TOF_THRESHOLD_MM && distance > 0) {
					current_state = STATE_RECORDING;
					chunk_count = 0;
					audio_idx = 0; // Reset our main float buffer index
					half_ready = 0;
					full_ready = 0;
					max_heard_volume = 0.0f;

					// Stop ToF Ranging so the I2C bus doesn't interrupt the audio processing
					VL53L1X_StopRanging(dev);

					// Tell the hardware DMA to start dumping microphone data into our array
					HAL_I2S_Receive_DMA(&hi2s2, i2s_rx_buf, I2S_DMA_SAMPLES);

					char msg[64];
					sprintf(msg, "--- ITEM DETECTED (%d mm) ---\n", distance);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
				}
			}
		} else if (current_state == STATE_RECORDING) {
			// The DMA finished filling the first half of the array
			if (half_ready) {
				half_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[0]);
				chunk_count++;
			}

			// The DMA finished filling the second half of the array
			if (full_ready) {
				full_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[HALF_SAMPLES]);
				chunk_count++;
			}

			// ---------------------------------------------------------
			// 1. SUCCESS: Did we finish collecting exactly 0.5s of audio?
			// ---------------------------------------------------------
			if (audio_idx >= AUDIO_LEN) {
				HAL_I2S_DMAStop(&hi2s2);

				// 1. Send Start Marker
				char start_msg[] = "START_AUDIO\n";
				HAL_UART_Transmit(&huart3, (uint8_t*) start_msg,
						strlen(start_msg), HAL_MAX_DELAY);

				// 2. Transmit each sample as a human-readable text string!
				// This completely immunizes the data from binary alignment corruption.
				for (int i = 0; i < AUDIO_LEN; i++) {
					int16_t pcm_val = (int16_t) (audio_buf[i] * 32767.0f);
					char val_str[16];

					// Convert the integer to text with a newline
					sprintf(val_str, "%d\n", pcm_val);
					HAL_UART_Transmit(&huart3, (uint8_t*) val_str,
							strlen(val_str), HAL_MAX_DELAY);
				}

				// 3. Send End Marker
				char end_msg[] = "END_AUDIO\n";
				HAL_UART_Transmit(&huart3, (uint8_t*) end_msg, strlen(end_msg),
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
			else if (chunk_count >= 250 && !is_impact_detected) {
				// Stop the microphone!
				HAL_I2S_DMAStop(&hi2s2);

				char msg[] = "TIMEOUT: Object detected, but no sound heard.\n\n";
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				// Only wait half a second on a timeout before scanning again
				HAL_Delay(500);
				VL53L1X_StartRanging(dev);
				is_impact_detected = 0;
				max_heard_volume = 0.0f;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_I2S2_Init(void) {

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
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_USB_OTG_FS_PCD_Init(void) {

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
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
	while (1) {
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
