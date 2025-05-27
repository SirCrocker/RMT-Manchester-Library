#pragma once

/* ----- INCLUDES ----- */
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----- MACROS ----- */

#define MANCH_RESOLUTION_HZ          1000000 // 1MHz resolution, 1 tick = 1us
#define MANCH_RX_GPIO_NUM            GPIO_NUM_20
#define MANCH_DECODE_MARGIN          9     // [us] Tolerance for parsing RMT symbols into bit stream
#define MANCH_BUFFER_SIZE            5000   // In packets
#define MANCH_HEADER                 0x08U

/* ----- TYPEDEFS & ENUMS ----- */

/**
 * @brief Type of combinations possible when sending manchester-encoded data
 */
typedef enum {
    SINGLE_SINGLE   = 0b011,  
    SINGLE_DOUBLE   = 0b010,
    DOUBLE_DOUBLE   = 0b110,
    DOUBLE_SINGLE   = 0b101,
    SINGLE_ENDSYM   = 0b111,
    SINGLE_INVALID  = 0b001,
    END_SYMBOL      = 0b100,
    INVALID_SYM     = 0b000,
} manch_logic_t;

/**
 * @brief Type of durations possible when sending manchester-encoded data
 */
typedef enum {
    SINGLE  = 0b001,
    DOUBLE  = 0b010,
    QUAD    = 0b101,
    INVALID = 0b000,
} manch_duration_t;

/**
 * @brief State of the communication
 */
typedef enum {
    HEADER,  /*!< Header state stays until the starting sequence is detected. */
    HEADER_DETECTED,  /*!< A header was detected but a change from header to data is pending */
    DATA,    /*!< In data state everything that is received is saved as valid information. */
    ENDING   /*!< Data state was ended and the communication ceased. */
} manch_state_t;

/* ---- Functions ----- */

/**
 * @brief Initialize the RMT receiver using Manchester encoding.
 */
esp_err_t manch_rmt_rx_init(void);

/**
 * @brief Check if data has been received
 * 
 * @return boolean indicating if data is available
 */
bool manch_rmt_rx_check_for_data(void);


/**
 * @brief Retrieve the data in the reception buffer
 * 
 * @param[out] value: value retrieved from the buffer
 * 
 * @return true if a value was retrieved
 */
bool manch_rmt_rx_retr_val(uint8_t **value);

#ifdef __cplusplus
}
#endif