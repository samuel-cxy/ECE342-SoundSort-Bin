#include <driver/i2s.h>

#define I2S_PORT I2S_NUM_0

#define PIN_I2S_BCLK 5
#define PIN_I2S_WS   6
#define PIN_I2S_SD   4

static int32_t i2s_samples[256];
static int16_t pcm16[256];

void setup() {
  Serial.begin(921600);
  delay(1000);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void loop() {
  size_t bytes_read = 0;
  esp_err_t err = i2s_read(I2S_PORT, i2s_samples, sizeof(i2s_samples), &bytes_read, portMAX_DELAY);
  if (err != ESP_OK) {
    return;
  }

  int count = bytes_read / sizeof(int32_t);

  for (int i = 0; i < count; i++) {
    int32_t s = i2s_samples[i];

    // Convert 32-bit mic sample down to signed 16-bit PCM
    pcm16[i] = (int16_t)(s >> 14);
  }

  Serial.write((uint8_t*)pcm16, count * sizeof(int16_t));
}