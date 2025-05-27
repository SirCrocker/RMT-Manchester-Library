#pragma once

/* ----- INCLUDES ----- */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "manch_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----- MACROS ----- */

#define MANCHESTER_RESOLUTION_HZ     1000000     // 1MHz resolution, 1 tick = 1us
#define MANCHESTER_TX_GPIO_NUM       GPIO_NUM_0  // As defined in the PCB
#define MANCHESTER_BAUD_RATE         BR_50KBD    // Maximum given by the PCB-defined bandwidth of the PV Panel

/* ----- FUNCTIONS ----- */

/**
 * @brief Initializes the RMT transmitters using Manchester encoding.
 * 
 * @returns status indicating the initialization of the transmitter.
 */
esp_err_t manch_rmt_tx_init(void);


/**
 * @brief Transmits the data in buffer through the initializes RMT channel.
 * 
 * @param buffer: buffer with he data to transmit (size limits?)
 * @param buffer_len: length of the buffer with data.
 * 
 * @returns status indicating the execution of the transmission.
 */
esp_err_t manch_rmt_tx_transmit(const uint8_t* buffer, size_t buffer_len);

#ifdef __cplusplus
}
#endif