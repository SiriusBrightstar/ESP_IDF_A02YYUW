/**
 * @file        a02yyuw.c
 * @author      SiriusBrightstar (siriusbrightstar@protonmail.com)
 * @brief       ESP-IDF Driver for A02YYUW Ultrasonic Distance Sensor
 * @version     0.1
 * @date        2024-02-10
 *
 * @copyright Copyright (c) SiriusBrightstar 2024
 *
 * MIT Licensed as described in the file LICENSE
 *
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "a02yyuw.h"

static const char *TAG = "A02YYUW";

esp_err_t a02yyuw_init(a02yyuw_dev_t *dev, uart_port_t uart_port,
                       gpio_num_t rx_gpio, gpio_num_t data_select_pin,
                       uint8_t data_select) {
  CHECK_ARG(dev);
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
  };
  ESP_ERROR_CHECK(uart_driver_install(uart_port, A02YYUW_RX_BUFFER_SIZE * 2, 0,
                                      0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(uart_port, GPIO_NUM_NC, rx_gpio, GPIO_NUM_NC, GPIO_NUM_NC));

  dev->uart_port = uart_port;
  dev->data_select = data_select;

  gpio_reset_pin(data_select_pin);
  gpio_set_direction(data_select_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(data_select_pin, data_select);

  // buffer for the incoming data
  dev->buf = (uint8_t *)malloc(A02YYUW_DATA_FRAME_SIZE + 1);
  if (!dev->buf)
    return ESP_ERR_NO_MEM;
  dev->last_value = -1;
  ESP_LOGD(TAG, "Successfully initialized UART driver for A02YYUW");
  return ESP_OK;
}

esp_err_t a02yyuw_deinit(a02yyuw_dev_t *dev) {
  CHECK_ARG(dev && dev->buf);

  free(dev->buf);
  dev->buf = NULL;
  uart_driver_delete(dev->uart_port);
  return ESP_OK;
}

esp_err_t a02yyuw_read_distance(a02yyuw_dev_t *dev, uint16_t *distance) {
  CHECK_ARG(dev && distance);
  memset(dev->buf, 0, A02YYUW_DATA_FRAME_SIZE + 1);

  *distance = -1;
  uart_flush(dev->uart_port);

  int len = uart_read_bytes(dev->uart_port, dev->buf, A02YYUW_DATA_FRAME_SIZE,
                            A02YYUW_RX_TIMEOUT / portTICK_PERIOD_MS);

  ESP_LOGD(TAG, "Data Frame Length : %d", len);
  ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_DEBUG);

  if ((len == A02YYUW_DATA_FRAME_SIZE) &&
      (dev->buf[0] == A02YYUW_DATA_FRAME_HEADER)) {
    if (dev->buf[3] == a02yyuw_calc_checksum(dev->buf)) {
      /* Combining Data High & Data Low Bytes */
      *distance = (dev->buf[1] << 8) | dev->buf[2];
      dev->last_value = *distance;
      ESP_LOGD(TAG, "Distance: %dmm", *distance);
      return ESP_OK;
    } else {
      ESP_LOGE(TAG, "%s", esp_err_to_name(ESP_ERR_INVALID_CRC));
      ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_ERROR);
      return ESP_ERR_INVALID_CRC;
    }
  } else {
    ESP_LOGE(TAG, "%s", esp_err_to_name(ESP_ERR_INVALID_RESPONSE));
    ESP_LOG_BUFFER_HEXDUMP(TAG, dev->buf, len, ESP_LOG_ERROR);
    return ESP_ERR_INVALID_RESPONSE;
  }
}

uint8_t a02yyuw_calc_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum = (data[0] + data[1] + data[2]) & 0x00FF;
  ESP_LOGD(TAG, "Checksum: %02x", checksum);

  return checksum;
}