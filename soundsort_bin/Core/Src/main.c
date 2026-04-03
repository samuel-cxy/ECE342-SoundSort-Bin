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
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "soundsort_model.h"
#include "VL53L1X_api.h"
#include "ssd1306.h"
#include "fonts.h"
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
#define TOF_THRESHOLD_MM 120

// Audio sample rate used by both STM32 and Python training
#define TARGET_SR 16000

// We store 0.5 seconds at 16kHz audio
// 16000 samples/sec * 0.5 sec = 8000 samples
#define AUDIO_LEN 8000

// FFT size used for spectral features
// Only the first 1024 audio samples are used for FFT features
#define N_FFT 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim4;

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
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float PI = 3.14159265358979f;

/* ---------------- Audio capture buffers ---------------- */
// Raw DMA buffer filled directly by I2S peripheral
static uint16_t i2s_rx_buf[I2S_DMA_SAMPLES];

// Final mono audio clip used for feature extraction
// This holds 8000 float samples = 0.5 seconds
static float audio_buf[AUDIO_LEN];

// Current write index into audio_buf[]
static uint32_t audio_idx = 0;

/* ---------------- FFT working buffers ---------------- */
// Input buffer for FFT (windowed first 1024 samples)
static float fft_in[N_FFT];

// Output power spectrum: for real FFT we only need bins 0..N/2
static float P[(N_FFT / 2) + 1];

// Internal real/imag arrays used by the FFT
// These are static to avoid large stack allocation
static float fft_real[N_FFT];
static float fft_imag[N_FFT];

/* ---------------- Runtime state/flag ---------------- */
// Set to 1 once we detect the actual impact sound
// Before that, we keep waiting even though recording has started
static uint8_t is_impact_detected = 0;

// Tracks the loudest sample seen in the current recording
static float max_heard_volume = 0.0f;

// Stores raw classifier scores (logits) for each class
// Useful for debugging / converting to probabilities
static float class_scores[NUM_CLASSES];

/* ---------------- DMA synchronization flags ---------------- */
// These flags are set inside DMA callbacks
// The main loop checks them and processes the correct half of the DMA buffer
volatile uint8_t half_ready = 0;
volatile uint8_t full_ready = 0;

/* ---------------- Audio preprocessing ---------------- */
// Running DC estimate for a simple DC blocker
// Helps remove microphone bias / offset
static int32_t dc_offset = 0;

/* ---------------- FSM States ---------------- */
typedef enum {
	STATE_IDLE,     // Waiting for an object to fall
	STATE_RECORDING // Object detected, microphone active, collecting impact audio
} SystemState;

SystemState current_state = STATE_IDLE;

// Counts how many DMA half/full chunks have been processed since recording started
uint32_t chunk_count = 0;

// I2C address for VL53L1X ToF sensor
uint16_t dev = 0x52;

// Compute power spectrum of a real-valued input signal
// Input: in[] = time-domain samples (length N_FFT)
// Output: P[] = power spectrum for bins 0..N_FFT/2
void compute_power_spectrum(float *in, float *P) {
	int N = N_FFT;

	/* --------------------------------------------------
	 * 1. Copy real input into complex FFT buffers
	 *    imag part starts at 0
	 * -------------------------------------------------- */
	for (int i = 0; i < N; i++) {
		fft_real[i] = in[i];
		fft_imag[i] = 0.0f;
	}

	/* --------------------------------------------------
	 * 2. Bit-reversal permutation
	 *    Reorders samples into the order required by the
	 *    iterative radix-2 FFT algorithm
	 * -------------------------------------------------- */
	int j = 0;
	for (int i = 0; i < N - 1; i++) {
		if (i < j) {
			float tr = fft_real[j];
			float ti = fft_imag[j];
			fft_real[j] = fft_real[i];
			fft_imag[j] = fft_imag[i];
			fft_real[i] = tr;
			fft_imag[i] = ti;
		}

		int k = N / 2;
		while (k <= j) {
			j -= k;
			k /= 2;
		}
		j += k;
	}

	/* --------------------------------------------------
	 * 3. Iterative radix-2 Cooley-Tukey FFT
	 *    Since N_FFT = 1024 = 2^10, we need 10 stages
	 * -------------------------------------------------- */
	for (int l = 1; l <= 10; l++) {
		int m = 1 << l;
		int m2 = m / 2;

		float omega_real = cosf(-2.0f * PI / m);
		float omega_imag = sinf(-2.0f * PI / m);

		for (int k = 0; k < N; k += m) {
			float w_real = 1.0f;
			float w_imag = 0.0f;

			for (int i = 0; i < m2; i++) {
				int t_idx = k + i + m2;
				float tr = w_real * fft_real[t_idx] - w_imag * fft_imag[t_idx];
				float ti = w_real * fft_imag[t_idx] + w_imag * fft_real[t_idx];

				int u_idx = k + i;
				fft_real[t_idx] = fft_real[u_idx] - tr;
				fft_imag[t_idx] = fft_imag[u_idx] - ti;
				fft_real[u_idx] += tr;
				fft_imag[u_idx] += ti;

				float next_w_real = w_real * omega_real - w_imag * omega_imag;
				w_imag = w_real * omega_imag + w_imag * omega_real;
				w_real = next_w_real;
			}
		}
	}

	/* --------------------------------------------------
	 * 4. Convert complex FFT result into power spectrum
	 *    Power = real^2 + imag^2
	 *    Only bins 0..N/2 are needed for real input
	 * -------------------------------------------------- */
	for (int k = 0; k <= N / 2; k++) {
		P[k] = fft_real[k] * fft_real[k] + fft_imag[k] * fft_imag[k];
	}
}

// Extract the same 15 features used in Python training
// feats[] layout:
// [0] RMS
// [1] Peak
// [2] Zero-crossing Rate
// [3] Decay ratio
// [4] Spectral centroid
// [5] Spectral rolloff
// [6] Active ratio
// [7]..[14] Band energies
void extract_features(float *x, float *feats) {
	/* --------------------------------------------------
	 * 0. Trim everything after the last "active" sample
	 *    This matches the Python preprocessing
	 *    If the impact ends early, the tail is zeroed out
	 * -------------------------------------------------- */
	int last_active_idx = -1;
	for (int i = 0; i < AUDIO_LEN; i++) {
		if (fabsf(x[i]) > 0.03f) {
			last_active_idx = i;
		}
	}

	if (last_active_idx >= 0) {
		for (int i = last_active_idx + 1; i < AUDIO_LEN; i++) {
			x[i] = 0.0f;
		}
	}

	/* --------------------------------------------------
	 * 1. Time-domain features
	 * -------------------------------------------------- */
	float sum_sq = 0.0f;  // for RMS
	float peak = 0.0f;    // max absolute amplitude
	float zcr = 0.0f;     // zero-crossing count
	int active_count = 0; // samples above active threshold

	for (int i = 0; i < AUDIO_LEN; i++) {
		float val = x[i];
		float abs_val = fabsf(val);

		sum_sq += val * val;

		if (abs_val > peak) {
			peak = abs_val;
		}

		// Count how much of the clip is "active"
		if (abs_val > 0.05f) {
			active_count++;
		}

		// Count sign changes for zero-crossing rate
		if (i > 0) {
			if ((x[i] >= 0.0f && x[i - 1] < 0.0f)
					|| (x[i] < 0.0f && x[i - 1] >= 0.0f)) {
				zcr += 1.0f;
			}
		}
	}

	// RMS = sqrt(mean(x^2))
	feats[0] = sqrtf(sum_sq / AUDIO_LEN + 1e-12f);

	// Peak amplitude
	feats[1] = peak + 1e-12f;

	// Zero-crossing rate
	feats[2] = zcr / AUDIO_LEN;

	// Compare energy in first third vs last third of the clip
	// If the sound decays quickly, first third energy will be much larger
	int third = AUDIO_LEN / 3;
	float e1 = 1e-12f;
	float e3 = 1e-12f;

	for (int i = 0; i < third; i++) {
		e1 += x[i] * x[i];
	}
	for (int i = AUDIO_LEN - third; i < AUDIO_LEN; i++) {
		e3 += x[i] * x[i];
	}

	feats[3] = logf(e1 / e3);

	/* --------------------------------------------------
	 * 2. Frequency-domain features
	 * -------------------------------------------------- */

	// Apply Hann window to the first 1024 samples before FFT
	for (int i = 0; i < N_FFT; i++) {
		/* FIX: use N_FFT - 1, not AUDIO_LEN - 1 */
		float window = 0.5f * (1.0f - cosf(2.0f * PI * i / (N_FFT - 1)));
		fft_in[i] = x[i] * window;
	}

	// Get power spectrum
	compute_power_spectrum(fft_in, P);

	// Total spectral energy
	float total_energy = 1e-12f;
	for (int k = 0; k <= N_FFT / 2; k++) {
		total_energy += P[k];
	}

	// Frequency represented by each FFT bin
	float bin_width = (float) TARGET_SR / (float) N_FFT;

	/* Spectral centroid:
	 * weighted average frequency
	 */
	float centroid_sum = 0.0f;
	for (int k = 0; k <= N_FFT / 2; k++) {
		float freq = k * bin_width;
		centroid_sum += freq * P[k];
	}
	feats[4] = centroid_sum / total_energy;

	/* Spectral rolloff:
	 * frequency below which 85% of energy lies
	 */
	float cumsum = 0.0f;
	float target = 0.85f * total_energy;
	float rolloff_freq = 0.0f;

	for (int k = 0; k <= N_FFT / 2; k++) {
		cumsum += P[k];
		if (cumsum >= target) {
			rolloff_freq = k * bin_width;
			break;
		}
	}
	feats[5] = rolloff_freq;

	// Fraction of samples whose magnitude exceeds 0.05
	feats[6] = (float) active_count / (float) AUDIO_LEN;

	/* Band energies:
	 * split the spectrum into 8 fixed frequency bands
	 * and normalize each by total energy
	 */
	const int edges[9] = { 0, 300, 700, 1200, 2000, 3000, 4500, 6500, 8000 };

	for (int b = 0; b < 8; b++) {
		float band_e = 0.0f;

		for (int k = 0; k <= N_FFT / 2; k++) {
			float freq = k * bin_width;
			if (freq >= edges[b] && freq < edges[b + 1]) {
				band_e += P[k];
			}
		}

		feats[7 + b] = band_e / total_energy;
	}
}

// Run logistic regression inference using the weights exported from Python
// For each class:
//   z = bias + sum(normalized_feature * weight)
// The class with the largest z is selected
int classify_garbage(float *feats) {
	int best_class = 0;
	float max_score = -1e9f;

	for (int c = 0; c < NUM_CLASSES; c++) {
		float z = LR_BIAS[c];

		for (int f = 0; f < NUM_FEATURES; f++) {
			// Normalize each feature using the same scaler that was used during training
			float scale = FEATURE_SCALE[f];
			if (scale < 1e-12f) {
				scale = 1.0f;
			}

			float norm_val = (feats[f] - FEATURE_MEAN[f]) / scale;
			z += norm_val * LR_WEIGHTS[c][f];
		}

		// Save logits for debugging / probability conversion
		class_scores[c] = z;

		if (z > max_score) {
			max_score = z;
			best_class = c;
		}
	}

	return best_class;
}

// Convert raw I2S DMA data into mono float samples and store them in audio_buf[]
// Notes:
// - The microphone stream arrives as 16-bit words in a larger I2S frame format
// - We take one sample every 4 words to get the desired mono channel
// - A simple DC blocker removes slow offset drift
// - We ignore the first few DMA chunks to let the stream settle
// - We only start copying once an actual impact is heard
static void fill_audio_buffer(uint16_t *src) {
	for (uint32_t i = 0; i < HALF_SAMPLES; i += 4) {
		// Extract one signed 16-bit sample
		int16_t raw_sample = (int16_t) src[i];

		// Simple DC blocker:
		// track the slow average, subtract it away
		dc_offset = dc_offset + ((raw_sample - dc_offset) / 64); // can adjust rate of change
		float sample_float = (float) (raw_sample - dc_offset) / 32768.0f;

		// Ignore the first few chunks after DMA starts
		// because they may contain startup junk / transient data
		if (chunk_count < 5) {
			continue;
		}

		float abs_val = fabsf(sample_float);

		// Track maximum amplitude for debugging
		if (abs_val > max_heard_volume) {
			max_heard_volume = abs_val;
		}

		// Impact detection:
		// once a sample crosses this threshold, start saving audio
		if (!is_impact_detected) {
			if (abs_val > 0.01f) { // Train in python uses 0.03
				is_impact_detected = 1;
			}
		}

		// Once impact starts, copy samples into the final 0.5s buffer
		if (is_impact_detected && audio_idx < AUDIO_LEN) {
			audio_buf[audio_idx++] = sample_float;
		}
	}
}

// OLED displays in IDLE state
void OLED_ShowIdle(void) {
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("SoundSort Bin", &Font_7x10);

	SSD1306_GotoXY(0, 20);
	SSD1306_Puts("Waiting...", &Font_11x18);

	SSD1306_UpdateScreen();
}

// OLED displays when timeout (triggers ToF but no input sound)
void OLED_ShowTimeout(void) {
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("No impact", &Font_11x18);

	SSD1306_GotoXY(0, 24);
	SSD1306_Puts("Try again", &Font_11x18);

	SSD1306_UpdateScreen();
}

// OLED displays the result: garbage type + confidence in probability
void OLED_ShowResult(const char *label, float prob_percent) {
	char prob_str[24];

	snprintf(prob_str, sizeof(prob_str), "Prob: %.2f%%", prob_percent);

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Result", &Font_7x10);

	SSD1306_GotoXY(0, 16);
	SSD1306_Puts((char*) label, &Font_11x18);

	SSD1306_GotoXY(0, 42);
	SSD1306_Puts(prob_str, &Font_7x10);

	SSD1306_UpdateScreen();
}

// Servo rotation functions
// Rotate servo to 0 degrees/reset
void servo_rotate_0(void) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
	HAL_Delay(2000);
}

// Rotate servo to 90 degrees/reset
void servo_rotate_90(void) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1166);
	HAL_Delay(2000);
}

// Rotate servo to 180 degrees/reset
void servo_rotate_180(void) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1832);
	HAL_Delay(2000);
}

// Rotate servo to 270 degrees/reset
void servo_rotate_270(void) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2500);
	HAL_Delay(2000);
}

// DMA Call Backs
// Ping-Pong buffering: while the CPU processes the first half, the DMA fills the second half
// first half of i2s_rx_buf[] is ready for processing
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == SPI2) {
		half_ready = 1;
	}
}

// second half of i2s_rx_buf[] is ready for processing
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
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	// Turn on RED LED to indicate boot is starting
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);

	/* ---------------- ToF Setup ---------------- */
	// 1. Reset ToF sensor (Pull XSHUT Low)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(10); // Hold it in reset

	// 2. Wake the ToF Sensor UP (Pull XSHUT High)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(50); // Give it time to boot internally

	// Wait until sensor reports it has finished booting
	uint8_t sensorState = 0;
	while (sensorState == 0) {
		VL53L1X_BootState(dev, &sensorState);
		HAL_Delay(2);
	}

	// 3. Configure ToF for short-range, fast updates
	VL53L1X_SensorInit(dev);
	VL53L1X_SetDistanceMode(dev, 1);
	VL53L1X_SetTimingBudgetInMs(dev, 33);
	VL53L1X_SetInterMeasurementInMs(dev, 33);
	VL53L1X_StartRanging(dev);

	/* ---------------- OLED Screen Setup ---------------- */
	if (SSD1306_Init() != HAL_OK) {
		char msg[] = "OLED init failed\r\n";
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	} else {
		OLED_ShowIdle();
	}

	/* ---------------- PWM/Servo Setup ---------------- */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	// Rotate to 0 degree
	servo_rotate_0();

	// /Close the lid
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
	HAL_Delay(2000);

	// Finish setup (successful)
	// red LED off, green LED on
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

	char msg[] = "All setup finished\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (current_state == STATE_IDLE) {
			// In IDLE state, repeatedly poll the ToF sensor
			// We only start audio capture when an object enters the bin
			OLED_ShowIdle();
			uint8_t dataReady = 0;
			VL53L1X_CheckForDataReady(dev, &dataReady);

			if (dataReady) {
				uint16_t distance = 0;
				VL53L1X_GetDistance(dev, &distance);
				VL53L1X_ClearInterrupt(dev);

				// If object is close enough, trigger recording
				if (distance < TOF_THRESHOLD_MM && distance > 0) {
					current_state = STATE_RECORDING;

					// Reset recording-related state
					chunk_count = 0;
					audio_idx = 0;
					half_ready = 0;
					full_ready = 0;

					is_impact_detected = 0;
					max_heard_volume = 0.0f;
					dc_offset = 0;
					memset(audio_buf, 0, sizeof(audio_buf));

					// Stop ToF while recording (ToF may stop working after a long time)
					VL53L1X_StopRanging(dev);

					// Start microphone DMA capture
					HAL_I2S_Receive_DMA(&hi2s2, i2s_rx_buf, I2S_DMA_SAMPLES);

					char msg[64];
					sprintf(msg, "--- ITEM DETECTED (%d mm) ---\r\n", distance);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
				}
			}
		} else if (current_state == STATE_RECORDING) {
			// In RECORDING state, the DMA callbacks set flags when either half of the DMA buffer is ready
			if (half_ready) {
				half_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[0]);
				chunk_count++;
			}

			// DMA finished filling the second half of the array
			if (full_ready) {
				full_ready = 0;
				fill_audio_buffer(&i2s_rx_buf[HALF_SAMPLES]);
				chunk_count++;
			}

			/* ------------------------------------------------------
			 * SUCCESS CASE:
			 * Once we have collected 8000 samples after impact,
			 * stop recording and run classification
			 * ------------------------------------------------------ */
			if (audio_idx >= AUDIO_LEN) {
				HAL_I2S_DMAStop(&hi2s2);

				uint32_t t0 = HAL_GetTick();

				float features[NUM_FEATURES];
				extract_features(audio_buf, features);

				int predicted_idx = classify_garbage(features);

				char msg[160];

				// Find actual time that feature extractions and classifications cost
				uint32_t t1 = HAL_GetTick();
				uint32_t proc_ms = t1 - t0;
				sprintf(msg, "Processing time: %lu ms\r\n", proc_ms);
				HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

				// For Debug through UART
				sprintf(msg, "\r\n=== CLASSIFICATION ===\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				sprintf(msg, "max amplitude: %.5f\r\n", max_heard_volume);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				for (int i = 0; i < NUM_FEATURES; i++) {
					sprintf(msg, "F[%02d] = %.6f\r\n", i, features[i]);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
				}

				for (int c = 0; c < NUM_CLASSES; c++) {
					sprintf(msg, "%s: %.6f\r\n", CLASS_NAMES[c],
							class_scores[c]);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
				}

				sprintf(msg, "\r\npredicted: %s\r\n",
						CLASS_NAMES[predicted_idx]);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				// Convert raw logits into softmax probabilities
				float max_logit = class_scores[0];
				for (int c = 1; c < NUM_CLASSES; c++) {
					if (class_scores[c] > max_logit) {
						max_logit = class_scores[c];
					}
				}

				float exp_sum = 0.0f;
				float probs[NUM_CLASSES];

				for (int c = 0; c < NUM_CLASSES; c++) {
					probs[c] = expf(class_scores[c] - max_logit);
					exp_sum += probs[c];
				}

				for (int c = 0; c < NUM_CLASSES; c++) {
					probs[c] /= exp_sum;
				}

				// Print probability for each class
				for (int c = 0; c < NUM_CLASSES; c++) {
					sprintf(msg, "%s prob: %.2f%%\r\n", CLASS_NAMES[c],
							probs[c] * 100.0f);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
				}

				sprintf(msg, "======================\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				// Display clasification result on OLED
				OLED_ShowResult(CLASS_NAMES[predicted_idx],
						probs[predicted_idx] * 100.0f);

				// Blink LED to show a classification happened
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);

				// Rotate servo to the corresponding class
				if (predicted_idx == 0) {
					servo_rotate_0(); // can
				}
				if (predicted_idx == 1) {
					servo_rotate_90(); // glass
				}
				if (predicted_idx == 2) {
					servo_rotate_180(); // paper
				}
				if (predicted_idx == 3) {
					servo_rotate_270(); // plastic
				}

				// Open the lid for 2sec and close it
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
				HAL_Delay(2000);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);

				// Resume ToF scanning for the next object
				VL53L1X_StartRanging(dev);
				is_impact_detected = 0;
				current_state = STATE_IDLE;

				// UART print ready
				sprintf(msg, "Ready!\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			}

			/* ------------------------------------------------------
			 * TIMEOUT CASE:
			 * If an object was detected but no real impact sound
			 * happened after enough DMA chunks, give up and reset.
			 * ------------------------------------------------------ */
			else if (chunk_count >= 250 && !is_impact_detected) {
				// Stop the microphone
				HAL_I2S_DMAStop(&hi2s2);

				char msg[] = "TIMEOUT: Object detected, but no sound heard.\n\n";
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				// Display on OLED
				OLED_ShowTimeout();

				// Only wait half a second on a timeout before scanning again
				HAL_Delay(500);
				VL53L1X_StartRanging(dev);
				is_impact_detected = 0;
				max_heard_volume = 0.0f;
				dc_offset = 0;
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 83;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 19999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

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

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
