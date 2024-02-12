/**
 * @file        a02yyuw.h
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

#ifndef __A02YYUW__
#define __A02YYUW__

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define A02YYUW_PIN_SELECT_REALTIME                                            \
  0 /* Real Time data - Response Time: 100ms */
#define A02YYUW_PIN_SELECT_PROCESSED                                           \
  1 /* Processed data - Response Time: 100-300ms */

#define A02YYUW_MIN_RANGE 30   /* Min value that can be sensed in mm */
#define A02YYUW_MAX_RANGE 4500 /* Max value that can be sensed in mm */

#define A02YYUW_RX_TIMEOUT 1000
#define A02YYUW_RX_BUFFER_SIZE 128
#define A02YYUW_DATA_FRAME_SIZE 4
#define A02YYUW_DATA_FRAME_HEADER 0xFF

/**
 * @brief Checks argument for validity
 *
 */
#define CHECK_ARG(VAL)                                                         \
  do {                                                                         \
    if (!(VAL))                                                                \
      return ESP_ERR_INVALID_ARG;                                              \
  } while (0)

/**
 * @brief Structure for A02YYUW Sensor
 */
typedef struct {
  uart_port_t uart_port; /* UART port used to communicate */
  uint8_t *buf;          /* Read buffer attached to this device */
  int16_t last_value;    /* Last read value */
  uint8_t data_select;   /* Choose between processed value or real time data */
} a02yyuw_dev_t;

/**
 * @brief               Initialize A02YYUW device descriptor
 *
 * @param dev           Pointer to the sensor device data structure
 * @param uart_port     UART port number
 * @param tx_gpio       UART Tx pin number
 * @param data_select   Data select pin number
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t a02yyuw_init(a02yyuw_dev_t *dev, uart_port_t uart_port,
                       gpio_num_t tx_gpio, gpio_num_t data_select_pin,
                       uint8_t data_select);

/**
 * @brief               Free A02YYUW device descriptor
 *
 * @param dev           Pointer to the sensor device data structure
 * @return esp_err_t    ESP_OK on success
 */
esp_err_t a02yyuw_deinit(a02yyuw_dev_t *dev);

/**
 * @brief                   Read distance data from sensor
 *
 * @param dev               Pointer to the sensor device data structure
 * @param[out] distance     Distance value in mm
 * @return esp_err_t        ESP_OK on success
 */
esp_err_t a02yyuw_read_distance(a02yyuw_dev_t *dev, uint16_t *distance);

/**
 * @brief           Calculate Checksum
 *
 * @param data      Buffer pointer
 *
 * @return uint8_t  Calculated Checksum value
 */
uint8_t a02yyuw_calc_checksum(uint8_t *data);

/**
 * @brief               (Private) Function to read a single byte
 *
 * @param dev           Pointer to the sensor device data structure
 * @param[out] data     1 Byte data value
 * @return esp_err_t    ESP_OK on success
 */

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif