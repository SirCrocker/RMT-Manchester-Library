#include "manch_rmt_tx.h"

/* ----- Static Variables ----- */
// For LOG module
static const char *TAG = "Manch TX RMT";

// RMT TX Channel
static rmt_channel_handle_t s_tx_channel = {0};
static rmt_encoder_handle_t s_manch_encoder = {0};

esp_err_t manch_rmt_tx_init(void) {
    ESP_LOGI(TAG, "create RMT TX channel");

    static rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = MANCHESTER_RESOLUTION_HZ,
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL * SOC_RMT_CHANNELS_PER_GROUP,    // We only have 1 tx channel, so lets use all the RAM
        // .mem_block_symbols = 128, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 16,  // number of transactions that allowed to pending in the background
        .gpio_num = MANCHESTER_TX_GPIO_NUM,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &s_tx_channel));

    ESP_LOGI(TAG, "install Manchester encoder");
    manch_encoder_config_t manch_encoder_cfg = {
        .resolution = MANCHESTER_RESOLUTION_HZ,
        .baud_rate = MANCHESTER_BAUD_RATE,
    };

    ESP_ERROR_CHECK(rmt_new_manch_encoder(&manch_encoder_cfg, &s_manch_encoder));

    ESP_LOGI(TAG, "enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(s_tx_channel));

    return ESP_OK;
}

esp_err_t manch_rmt_tx_transmit(const uint8_t* buffer, size_t buffer_len) {
    
    // so we don´t send frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // loop
    };

    const vlc_manch_data_t manch_data = {
        .data = buffer,
        .data_size = buffer_len,
    };

    ESP_LOGI(TAG, "transmit buffer with size: %d ", manch_data.data_size);
    ESP_ERROR_CHECK(rmt_transmit(s_tx_channel, s_manch_encoder, &manch_data, sizeof(manch_data), &transmit_config));
    ESP_LOGI(TAG, "successfully transmitted buffer");
    return ESP_OK;
}