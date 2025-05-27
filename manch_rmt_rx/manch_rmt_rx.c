/* Based on the NEC decoder by Espressif. */
#include "manch_rmt_rx.h"
#include "thesis-vars.h"
#include "string.h"

// #define PACKET_LEN  10

/* ----- Static Variables ----- */
// For log module
static const char* TAG = "Manch RX RMT";

// Data-buffer-related variables
static size_t s_pbuffer_read  = 0;                 /*!< Pointer to the read position in the buffer */
static size_t s_pbuffer_write = 0;                 /*!< Pointer to the write position in the buffer */
static uint8_t s_pbuffer[MANCH_BUFFER_SIZE][PACKET_SIZE] = {0}; /*!< Buffer with the received bytes */

// RMT and receiver buffer
static rmt_channel_handle_t s_rx_channel = NULL;   /*!< this way we can access the rx channel handle from everywhere */
static QueueHandle_t s_receive_queue;              /*!< this way we can access the rx channel queue from everywhere */
static rmt_receive_config_t s_receive_config;      /*!< I do not know if the config is so relevant, but better to have it as a global var */
static rmt_symbol_word_t s_raw_symbols[10000];     /*!< Save the received RMT symbols. One symbol < 2 bits */

/* ---- Static Functions ----- */

/**
 * @brief Check whether a duration is within the expected range. The range
 * has the duration tolerance added inside the function.
 * 
 * @param[in] signal_duration: the measured duration of the signal
 * @param[in] spec_duration: the expected (ideal) duration of the signal.
 * 
 * @see the macro for the duration tolerance.
 */
static inline bool manch_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + MANCH_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - MANCH_DECODE_MARGIN));
}

/**
 * @brief Check whether a duration is within a expected range. The range
 * minimum and maximum values have to be given to the function
 * 
 * @param[in] signal_duration: the measured duration of the signal
 * @param[in] min_duration: the signal minimum duration
 * @param[in] max_duration: the signal maximum duration
 * 
 */
static inline bool manch_check_in_range_defined(uint32_t signal_duration, uint32_t min_duration, uint32_t max_duration)
{
    return (signal_duration < max_duration) &&
           (signal_duration > min_duration);
}

/**
 * @brief Determines if the duration of a level is a valid manchester symbol. Possible
 * values are SINGLE (1 period), DOUBLE (2 periods), QUAD (4 periods; used for indicating end),
 * and INVALID (time less than one period). THE TIMINGS ARE ADJUSTED FOR SINGLE AND DOUBLE.
 * 
 * @param[in] signal_duration 
 * 
 * @return manch_duration_t that indicates the signal duration
 */
static manch_duration_t manch_duration_defined(uint32_t signal_duration) {
    if (manch_check_in_range_defined(signal_duration, 15, 24)) { // Single period
        return SINGLE;
    } else if (manch_check_in_range_defined(signal_duration, 26, 52)) {  // Double period
        return DOUBLE;
    } else if (manch_check_in_range(signal_duration, 80)) { // Quadruple period
        return QUAD;
    }
    
    return INVALID; // Invalid symbol (too long, too short, or out of spec)
}


/**
 * @brief Determines if the duration of a level is a valid manchester symbol. Possible
 * values are SINGLE (1 period), DOUBLE (2 periods), QUAD (4 periods; used for indicating end),
 * and INVALID (time less than one period).
 * 
 * @param[in] signal_duration 
 * 
 * @return manch_duration_t that indicates the signal duration
 */
static manch_duration_t manch_duration(uint32_t signal_duration) {
    if (manch_check_in_range(signal_duration, 20)) { // Single period
        return SINGLE;
    } else if (manch_check_in_range(signal_duration, 40)) {  // Double period
        return DOUBLE;
    } else if (manch_check_in_range(signal_duration, 80)) { // Quadruple period
        return QUAD;
    }
    
    return INVALID; // Invalid symbol (too long, too short, or out of spec)
}


/**
 * @brief Get type of manchester logic symbol given the duration of the on and off
 * states included in an rmt symbol.a64l
 * 
 * @param[in] rmt_manch_symbols: pointer to an rmt symbol used for manchester
 * 
 * @return manchester logic symbol type
 */
static manch_logic_t manch_parse_logic(rmt_symbol_word_t *rmt_manch_symbols)
{
    manch_duration_t dur0 = manch_duration(rmt_manch_symbols->duration0);
    manch_duration_t dur1 = manch_duration(rmt_manch_symbols->duration1);

    // TODO: IF dur0 is VALID!!!!!

    if (dur0 != INVALID && dur1 == INVALID) {
        return SINGLE_INVALID;
    }


    if (dur0 == INVALID || dur1 == INVALID) {
        return INVALID_SYM;
    }

    return (manch_logic_t)((dur0 << 1) | dur1); // TODO: Valid? (apparently yes)
}


/**
 * @brief save a value to the received uint8_t buffer
 * 
 * @param[in] value: value to save
 */
static inline void save_value_to_buffer(uint8_t value, bool reset) {
    static size_t internal_packet_w = 0;

    if (!reset) {
        s_pbuffer[s_pbuffer_write][internal_packet_w] = value;
        internal_packet_w = (internal_packet_w + 1) % PACKET_SIZE;
        ESP_LOGI(TAG, "Buffer used: [%d][%d]\n", s_pbuffer_write, internal_packet_w - 1);
    }

    bool update_buffer = reset ^ (internal_packet_w == 0);
    if (update_buffer) {
        s_pbuffer_write = (s_pbuffer_write + 1) % MANCH_BUFFER_SIZE;
        // Here we should memset the next buffer sector to all 0s to avoid crossing information [In case we need to decrease the buffer size]
        memset(s_pbuffer[s_pbuffer_write], 0, sizeof(uint8_t) * PACKET_SIZE);
        internal_packet_w = 0;
    }
    
    return;
}


/**
 * @brief Decode Manchester-encoded RMT symbols and place them into a buffer
 * to later retrieve the data decoded.
 * 
 * @param[in] rmt_manch_symbols: manchester-encoded rmt symbol
 * 
 * @return bool indicating if data was written.
 */
static bool manch_parse_frame(rmt_symbol_word_t *rmt_manch_symbols)
{
    ESP_LOGI(TAG, "parse received symbols as manchester frames");
    rmt_symbol_word_t *cur_symbol = rmt_manch_symbols;
    size_t symbol_num = -1; // For overflow purposes
    manch_state_t state = HEADER;
    bool manch_carry_zero = false;

    uint8_t temp_var = 0;
    uint8_t byte_idx = 0;

    // header (we look for )
    ESP_LOGI(TAG, "looking for header");
    uint8_t sequence = 0;
    while (state == HEADER || state == HEADER_DETECTED) {
        customYield(); // Yield so watchdog does not kill us>
        symbol_num++;
        ESP_LOGI(TAG, "to process symbol num %d", symbol_num);
        manch_logic_t parsed = manch_parse_logic(cur_symbol);
        // manch_logic_t header[] = {
        //     DOUBLE_DOUBLE,
        //     SINGLE_SINGLE,
        //     DOUBLE_SINGLE
        // };

        // ESP_LOGI(TAG, "symbol manch logic: %03x -- seq: %d", (uint8_t)parsed, sequence);

        switch (sequence)
        {
        case 0:
            if (parsed == DOUBLE_DOUBLE) { 
                sequence++; 
                cur_symbol++; 
            } else if (state == HEADER_DETECTED) {
                state = DATA;
                ESP_LOGI(TAG, "move to data");
            } else {
                temp_var++;
                cur_symbol++;
                if (temp_var >= 4) {
                    ESP_LOGI(TAG, "unable to detect header. Decoding cancelled");
                    return false;
                }
            }
            continue;
            break;
        
        case 1:
            if (parsed == SINGLE_SINGLE) { sequence++; cur_symbol++; }
            else {sequence = 0; cur_symbol++;}
            continue;
            break;

        case 2:
            if (parsed == DOUBLE_SINGLE) {
                ESP_LOGI(TAG, "header detected");
                state = HEADER_DETECTED;
                sequence = 0;
                cur_symbol++;
            } else if (parsed == DOUBLE_DOUBLE) {
                ESP_LOGI(TAG, "header and zero carry detected");  // If we need to carry zero, we have detected data
                state = DATA;
                manch_carry_zero = true;
                sequence = 0;
                cur_symbol++;
            } else {
                sequence = 0;
                cur_symbol++;
            }
            continue;
            break;

        default:
            sequence = 0;
            cur_symbol++;
            break;
        }
        
    }

    // DATA
    temp_var = 0;
    ESP_LOGI(TAG, "decode data");
    while (state == DATA)
    {
        customYield(); // Yield so watchdog does not kill us>
        ESP_LOGI(TAG, "to process symbol num %d | carry zero: %s", symbol_num, manch_carry_zero ? "true" : "false");
        manch_logic_t parsed_symbol = manch_parse_logic(cur_symbol);

        switch (parsed_symbol)
        {
        case SINGLE_DOUBLE: // short-long
            ESP_LOGI(TAG, "detected single-double");
            if (manch_carry_zero) {
                ESP_LOGI(TAG, "carry zero hope fix activated");
                // For BER purposes lets process this is a double-single (110 -> 100)
                // Maybe the data slicer transitioned wrongly
                // HOPE FIX
                temp_var |= 1 << byte_idx; // save one
                byte_idx = (byte_idx + 1) % 8;

                if (byte_idx == 0) {
                    save_value_to_buffer(temp_var, false);
                    temp_var = 0;
                }

                temp_var &= ~(1 << byte_idx); // save zero
                manch_carry_zero = false;
                
                // return false; // non-possible state
            }
            else {
                temp_var &= ~(1 << byte_idx); // save zero
                manch_carry_zero = true;
            }
            break;
        
        case SINGLE_SINGLE: // short-short
            ESP_LOGI(TAG, "detected single-single");
            if (manch_carry_zero) {
                temp_var |= 1 << byte_idx; // save one
                // manch_carry_zero = true; 
                break;
            }

            temp_var &= ~(1 << byte_idx); // save zero
            // manch_carry_zero = false;
            break;

        case DOUBLE_SINGLE: // long-short
            ESP_LOGI(TAG, "detected double-single");
            if (manch_carry_zero) {
                temp_var |= 1 << byte_idx; // save one
                byte_idx = (byte_idx + 1) % 8;

                if (byte_idx == 0) {
                    save_value_to_buffer(temp_var, false);
                    temp_var = 0;
                }

                temp_var &= ~(1 << byte_idx); // save zero
                manch_carry_zero = false;
            } else {
                ESP_LOGI(TAG, "reached invalid state");
                return false; // invalid state /* It is hard for a single to convert to double */
            }
            break;

        case DOUBLE_DOUBLE: // long-long
            ESP_LOGI(TAG, "detected double-double");
            if (manch_carry_zero) {
                temp_var |= 1 << byte_idx; // save one
                byte_idx = (byte_idx + 1) % 8;

                if (byte_idx == 0) {
                    save_value_to_buffer(temp_var, false);
                    temp_var = 0;
                }

                temp_var &= ~(1 << byte_idx); // save zero
                manch_carry_zero = true;
            } else {
                ESP_LOGI(TAG, "reached invalid state");
                return false; // invalid state /* It is hard for two singles to convert to doubles */
            }
            break;

        case SINGLE_INVALID: // TODO TODO TODO
            ESP_LOGI(TAG, "detected single invalid. (goto single endsym)");
            // fall through
        case SINGLE_ENDSYM:
            ESP_LOGI(TAG, "detected endsym");
            if (manch_carry_zero) {
                temp_var |= 1 << byte_idx; // save one
                // manch_carry_zero = true; 
            }
            // fall through
        case END_SYMBOL:
            ESP_LOGI(TAG, "ending");
            state = ENDING;
            break;

        case INVALID:
        default:
            ESP_LOGI(TAG, "invalid");
            state = ENDING;
            break;
        }
    
        cur_symbol++;
        symbol_num++;
        byte_idx = (byte_idx + 1) % 8;
        
        if (byte_idx == 0) {
            save_value_to_buffer(temp_var, false);
            temp_var = 0;
        }

    }

    save_value_to_buffer(0x00, true);
    return s_pbuffer_read != s_pbuffer_write;
}


/** 
 * @brief Decode RMT symbols into Manchester and save the received data into a buffer
 * 
 * @param[in] rmt_manch_symbols: pointer to the buffer where the RMT symbols are stored
 * @param[in] symbol_num: number of symbols in the buffer
 * 
 * @returns bool indicating if the symbols were parsed
 * 
 */
static bool parse_manchester_rmt_symbols(rmt_symbol_word_t *rmt_manch_symbols, size_t symbol_num)
{
    ESP_LOGI(TAG, "parse received symbols");
    
    if (symbol_num < 5) {
        return false;
    }

    #ifdef DEBUG_PROJECT
    printf("MANCH frame start---\r\n");
        for (size_t i = 0; i < symbol_num; i++) {
            printf("{%d:%d},{%d:%d}\r\n", rmt_manch_symbols[i].level0, rmt_manch_symbols[i].duration0,
                rmt_manch_symbols[i].level1, rmt_manch_symbols[i].duration1);
        }
    printf("---MANCH frame end\n");
    #endif 

    rmt_symbol_word_t *symbols_to_parse = NULL;
    symbols_to_parse = rmt_manch_symbols;
    rmt_symbol_word_t *p_displacedSymbols = NULL;

    //!< In case that the receiver default mode is ON
    if (rmt_manch_symbols->level0 == 0) {
        ESP_LOGI(TAG, "symbols displaced");
        // Displace symbols by one
        // printf("Memory before malloc %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        p_displacedSymbols = (rmt_symbol_word_t *)malloc(sizeof(rmt_symbol_word_t) * symbol_num);
        // printf("Memory after malloc %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (p_displacedSymbols == NULL) {
            ESP_LOGE(TAG, "NO MEMORY AVAILABLE %lu\n", esp_get_free_heap_size());
        }

        for (size_t idx = 0; idx < symbol_num; idx++) {
            p_displacedSymbols[idx].val = rmt_manch_symbols[idx].val >> 16;

            if ((idx + 1) != symbol_num) {
                p_displacedSymbols[idx].val |= (rmt_manch_symbols[idx + 1].val << 16);
            }

        }

        symbols_to_parse = p_displacedSymbols;

        #ifdef DEBUG_PROJECT
        printf("MANCH frame start displaced---\r\n");
            for (size_t i = 0; i < symbol_num; i++) {
                printf("{%d:%d},{%d:%d}\r\n", symbols_to_parse[i].level0, symbols_to_parse[i].duration0,
                    symbols_to_parse[i].level1, symbols_to_parse[i].duration1);
            }
        printf("---MANCH frame end displaced\n");
        #endif 

    }

    // decode RMT symbols
    if (manch_parse_frame(symbols_to_parse)) {
        // save_value_to_buffer(0x00, true);

        if (p_displacedSymbols) {
            free(p_displacedSymbols);
        }

        ESP_LOGI(TAG, "symbols parsed");
        return true;
    }
    
    // save_value_to_buffer(0x00, true);

    if (p_displacedSymbols) {
        free(p_displacedSymbols);
    }

    ESP_LOGI(TAG, "symbols could not be parsed");
    return false;

}

/**
 * @brief Callback to be called when the RX transaction finalizes
 * 
 * @param[in] channel: RMT channel handle
 * @param[in] edata: done event data
 * @param[in] user_data: user data
 * 
 * @return bool indicating if symbols were received
 */
static IRAM_ATTR bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}


/* ----- External Functions ----- */


bool manch_rmt_rx_retr_val(uint8_t **value) {

    if (s_pbuffer_read != s_pbuffer_write) {
        *value = s_pbuffer[s_pbuffer_read];
        ESP_LOGI(TAG, "Buffer read: %d\n", s_pbuffer_read);
        s_pbuffer_read = (s_pbuffer_read + 1) % MANCH_BUFFER_SIZE;
        return true;
    }

    *value = NULL;
    return false;
}


esp_err_t manch_rmt_rx_init(void)
{
    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = GPIO_NUM_20,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = MANCH_RESOLUTION_HZ,
        .mem_block_symbols = 80, // amount of RMT symbols that the channel can store at a time
        .intr_priority = 2,
    };
    
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &s_rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    s_receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(s_receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rx_channel, &cbs, s_receive_queue));

    // the following timing requirement is based on Manchester at 50 kBd
    s_receive_config = (rmt_receive_config_t){
        .signal_range_min_ns = 1250,     // the shortest duration for Manchester signal is 20us, 1250ns (1.25 us) < 20us, valid signal won't be treated as noise
        .signal_range_max_ns = 100000, // the longest duration for Manchester signal is 40us, 300000ns (300us) > 40us, the receive won't stop early
    };

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(s_rx_channel));

    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(s_rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &s_receive_config));

    return ESP_OK;
}


bool manch_rmt_rx_check_for_data(void) {
    bool ret = false;

    ESP_LOGI(TAG, "check for available data");
    rmt_rx_done_event_data_t rx_data;

    // wait for RX done signal
    while (xQueueReceive(s_receive_queue, &rx_data, pdMS_TO_TICKS(10)) == pdPASS) {
        // parse the receive symbols and print the result
        ret = parse_manchester_rmt_symbols(rx_data.received_symbols, rx_data.num_symbols);
        // start receive again
        ESP_ERROR_CHECK(rmt_receive(s_rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &s_receive_config));
    }

    return ret;
}