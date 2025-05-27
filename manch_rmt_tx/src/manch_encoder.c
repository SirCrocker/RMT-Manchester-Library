/* Based on Espressif's NEC encoder example */

#include "esp_check.h"
#include "manch_encoder.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "manch_encoder";

typedef struct {
    rmt_encoder_t base;           // the base "class", declares the standard encoder interface
    rmt_encoder_t *copy_encoder;  // use the copy_encoder to encode the leading and ending pulse
    rmt_encoder_t *bytes_encoder; // use the bytes_encoder to encode the address and command data
    rmt_symbol_word_t *header_symbol; // Manchester header code with RMT representation
    size_t header_len;  // Number of RMT symbols in the header
    rmt_symbol_word_t trailer_symbol_opt1;  // Manchester trailer code opt 1 with RMT representation
    rmt_symbol_word_t trailer_symbol_opt2;  // Manchester trailer code opt 2 with RMT representation
    int state;
} rmt_manch_encoder_t;

static size_t rmt_encode_manch(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ESP_LOGI(TAG, "transmit RMT Manchester data");
    rmt_manch_encoder_t *manch_encoder = __containerof(encoder, rmt_manch_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    vlc_manch_data_t *tx_buffer = (vlc_manch_data_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = manch_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = manch_encoder->bytes_encoder;

    switch (manch_encoder->state) {
    case 0: // send header
        ESP_LOGI(TAG, "transmit header");
        for (uint8_t i = 0; i < MANCH_HEADER_REPETITIONS; i++)
        {
            for (size_t j = 0; j < manch_encoder->header_len; j++)
            {
                // ESP_LOGI(TAG, "encoded symbools: %d", encoded_symbols);
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &manch_encoder->header_symbol[j],
                                                        sizeof(rmt_symbol_word_t), &session_state);
            }
            
        }

        ESP_LOGI(TAG, "number of symbols encoded %d", encoded_symbols);
        
        if (session_state & RMT_ENCODING_COMPLETE) {
            manch_encoder->state = 1; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 1: // send data
        ESP_LOGI(TAG, "transmit data");
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, tx_buffer->data, sizeof(uint8_t) * tx_buffer->data_size, &session_state);
        ESP_LOGI(TAG, "number of symbols encoded %d", encoded_symbols);
        if (session_state & RMT_ENCODING_COMPLETE) {
            manch_encoder->state = 2; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            ESP_LOGI(TAG, "encoder memory full");
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 2: // send trailer
        ESP_LOGI(TAG, "transmit trailer");
        rmt_symbol_word_t *ending_symbol = (tx_buffer->data[tx_buffer->data_size - 1] & 0x80) ? 
                                          &manch_encoder->trailer_symbol_opt2 : &manch_encoder->trailer_symbol_opt1;

        encoded_symbols += copy_encoder->encode(copy_encoder, channel, ending_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        ESP_LOGI(TAG, "number of symbols encoded %d", encoded_symbols);
        if (session_state & RMT_ENCODING_COMPLETE) {
            manch_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    }
out:
    *ret_state = state;
    ESP_LOGI(TAG, "transmission ended");
    return encoded_symbols;
}

static esp_err_t rmt_del_manch_encoder(rmt_encoder_t *encoder)
{
    ESP_LOGI(TAG, "delete manch encoder");
    rmt_manch_encoder_t *manch_encoder = __containerof(encoder, rmt_manch_encoder_t, base);
    rmt_del_encoder(manch_encoder->copy_encoder);
    rmt_del_encoder(manch_encoder->bytes_encoder);
    free(manch_encoder);
    return ESP_OK;
}

static esp_err_t rmt_manch_encoder_reset(rmt_encoder_t *encoder)
{
    ESP_LOGI(TAG, "reset manch encoder");
    rmt_manch_encoder_t *manch_encoder = __containerof(encoder, rmt_manch_encoder_t, base);
    rmt_encoder_reset(manch_encoder->copy_encoder);
    rmt_encoder_reset(manch_encoder->bytes_encoder);
    manch_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

IRAM_ATTR esp_err_t rmt_new_manch_encoder(const manch_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    ESP_LOGI(TAG, "create manchester encoder");
    esp_err_t ret = ESP_OK;
    rmt_manch_encoder_t *manch_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    manch_encoder = rmt_alloc_encoder_mem(sizeof(rmt_manch_encoder_t));
    ESP_GOTO_ON_FALSE(manch_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for manch encoder");
    manch_encoder->base.encode = rmt_encode_manch;
    manch_encoder->base.del = rmt_del_manch_encoder;
    manch_encoder->base.reset = rmt_manch_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &manch_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    ESP_LOGI(TAG, "set header and trailer information");
    // timings
    uint16_t symbol_duration = (uint16_t)config->baud_rate;

    uint16_t _duration_single = symbol_duration * config->resolution / 1000000;
    uint16_t _duration_double = _duration_single * 2;
    
    manch_encoder->header_len = 3;
    manch_encoder->header_symbol = (rmt_symbol_word_t*) malloc(sizeof(rmt_symbol_word_t) * 3);

    manch_encoder->header_symbol[0] = (rmt_symbol_word_t) {   
        .level0 = 1, .duration0 = _duration_double,
        .level1 = 0, .duration1 = _duration_double
    };

    manch_encoder->header_symbol[1] = (rmt_symbol_word_t) {   
        .level0 = 1, .duration0 = _duration_single,
        .level1 = 0, .duration1 = _duration_single
    };

    manch_encoder->header_symbol[2] = (rmt_symbol_word_t) {   
        .level0 = 1, .duration0 = _duration_double,
        .level1 = 0, .duration1 = _duration_single
    };
    
    // construct the ending code with RMT symbol format    
    manch_encoder->trailer_symbol_opt1 = (rmt_symbol_word_t) { // Option if last symbol ends in low
        .level0 = 1,
        .duration0 = symbol_duration * 4 * config->resolution / 1000000,
        .level1 = 0,
        .duration1 = symbol_duration * 14 * config->resolution / 1000000,
    };

    manch_encoder->trailer_symbol_opt2 = (rmt_symbol_word_t) { // Option if last symbol ends in high
        .level0 = 0,
        .duration0 = symbol_duration * 4 * config->resolution / 1000000,
        .level1 = 1,
        .duration1 = symbol_duration * 14 * config->resolution / 1000000,
    };

    rmt_bytes_encoder_config_t bytes_encoder_config = { // IEEE802.3 Convention
        .bit0 = {
            .level0 = 1,
            .duration0 = symbol_duration * config->resolution / 1000000, // T0H=20us
            .level1 = 0,
            .duration1 = symbol_duration * config->resolution / 1000000, // T0L=20us
        },
        .bit1 = {
            .level0 = 0,
            .duration0 = symbol_duration * config->resolution / 1000000,  // T1H=20us
            .level1 = 1,
            .duration1 = symbol_duration * config->resolution / 1000000, // T1L=20us
        },
        .flags.msb_first = 0,
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &manch_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");

    *ret_encoder = &manch_encoder->base;
    return ESP_OK;
err:
    if (manch_encoder) {
        if (manch_encoder->bytes_encoder) {
            rmt_del_encoder(manch_encoder->bytes_encoder);
        }
        if (manch_encoder->copy_encoder) {
            rmt_del_encoder(manch_encoder->copy_encoder);
        }
        if (manch_encoder->header_symbol) {
            free(manch_encoder->header_symbol);
        }
        free(manch_encoder);
    }
    return ret;
}