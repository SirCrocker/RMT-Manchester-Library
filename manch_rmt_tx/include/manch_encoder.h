/*
 * BASED ON: Espressif Systems' IR NEC RMT EXAMPLE
 */
#pragma once

/************************** 
-- INCLUDES -- 
**************************/

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************** 
-- MACROS -- 
**************************/

#define MANCH_HEADER_REPETITIONS 4 //!< The number of times that the header is sent


/************************** 
-- STRUCTS & ENUMS -- 
**************************/

typedef struct
{
    const uint8_t* data;
    size_t data_size;
} vlc_manch_data_t;

/**
 * @brief Baud rates tested with the system. They are chosen so the symbol period is a multiple of 1 us.
 */
typedef enum baudRates {
    BR_20KBD = 50,  // in us
    BR_50KBD = 20,  // in us
} manch_baud_rate_t;

/**
 * @brief Type of IR NEC encoder configuration
 */
typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
    manch_baud_rate_t baud_rate; /*!< Baud Rate, unitless */
} manch_encoder_config_t;

/************************** 
-- FUNCTIONS -- 
**************************/

/**
 * @brief Create RMT encoder for encoding manchester into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating IR NEC encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_manch_encoder(const manch_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);


#ifdef __cplusplus
}
#endif