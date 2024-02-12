#include "a02yyuw.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>

static const uint8_t RX_PIN = 16;
static const uint8_t DATA_SELECT_PIN = 17;

const char *TAG = "MAIN";

void app_main(void) {
  uint16_t distance = 0;
  a02yyuw_dev_t dev;

  ESP_ERROR_CHECK(a02yyuw_init(&dev, UART_NUM_2, RX_PIN, DATA_SELECT_PIN,
                               A02YYUW_PIN_SELECT_PROCESSED));

  while (true) {
    ESP_ERROR_CHECK(a02yyuw_read_distance(&dev, &distance));
    ESP_LOGI(TAG, "Distance: %dmm", distance);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}