#include "ow_device.hpp"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/rmt_types.h"
#include "soc/clk_tree_defs.h"
#include <array>
#include <cstdint>
#include <span>
#include <utility>

static std::uint8_t rx_queue_storage[sizeof(rmt_rx_done_event_data_t)];
static StaticQueue_t rx_queue_data;

static bool IRAM_ATTR rx_event_callback(rmt_channel_handle_t channel,
                                        rmt_rx_done_event_data_t const* data,
                                        void* ctx) noexcept
{
    BaseType_t task_woken{pdFALSE};
    xQueueSendFromISR(static_cast<QueueHandle_t>(ctx), data, &task_woken);
    return task_woken == pdTRUE;
}

namespace esp32_utility {

    void OWDevice::initialize(this OWDevice& self, gpio_num_t const gpio_num) noexcept
    {
        self.initialize_rx(gpio_num);
        self.initialize_tx(gpio_num);
        self.initialized = true;
    }

    void OWDevice::deinitialize(this OWDevice& self) noexcept
    {
        self.deinitialize_rx();
        self.deinitialize_tx();
        self.initialized = false;
    }

    void OWDevice::write8(this OWDevice& self, std::uint8_t const data) noexcept
    {
        if (self.initialized) {
            static rmt_transmit_config_t tx_config{};
            tx_config.flags.eot_level = BUS_RELEASED;

            ESP_ERROR_CHECK(rmt_transmit(self.tx_channel, self.bytes_encoder, &data, 1, &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(self.tx_channel, RMT_TIMEOUT_MS));
        }
    }

    void OWDevice::write32(this OWDevice& self, std::uint32_t const data) noexcept
    {
        if (self.initialized) {
            static rmt_transmit_config_t tx_config{};
            tx_config.flags.eot_level = BUS_RELEASED;

            ESP_ERROR_CHECK(rmt_transmit(self.tx_channel, self.bytes_encoder, &data, 4, &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(self.tx_channel, RMT_TIMEOUT_MS));
        }
    }

    void OWDevice::write16(this OWDevice& self, std::uint16_t const data) noexcept
    {
        if (self.initialized) {
            static rmt_transmit_config_t tx_config{};
            tx_config.flags.eot_level = BUS_RELEASED;

            ESP_ERROR_CHECK(rmt_transmit(self.tx_channel, self.bytes_encoder, &data, 1, &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(self.tx_channel, RMT_TIMEOUT_MS));
        }
    }

    void OWDevice::write64(this OWDevice& self, std::uint64_t const data) noexcept
    {
        if (self.initialized) {
            static rmt_transmit_config_t tx_config{};
            tx_config.flags.eot_level = BUS_RELEASED;

            ESP_ERROR_CHECK(rmt_transmit(self.tx_channel, self.bytes_encoder, &data, 8, &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(self.tx_channel, RMT_TIMEOUT_MS));
        }
    }

    std::uint8_t OWDevice::read8(this OWDevice& self) noexcept
    {
        if (self.initialized) {
            static rmt_receive_config_t rx_config{};
            rx_config.signal_range_min_ns = RX_MIN_NS;
            rx_config.signal_range_max_ns = (TIMING_A + TIMING_B) * 1000;

            ESP_ERROR_CHECK(rmt_receive(self.rx_channel, self.rx_buffer, sizeof(self.rx_buffer), &rx_config));

            self.write16(0xFF);

            rmt_rx_done_event_data_t event;
            if (xQueueReceive(self.rx_queue, &event, pdMS_TO_TICKS(RMT_TIMEOUT_MS))) {
                return parse_symbols(std::span{event.received_symbols, event.num_symbols});
            }
        }
        std::unreachable();
    }

    std::uint16_t OWDevice::read16(this OWDevice& self) noexcept
    {
        return std::uint16_t();
    }

    std::uint32_t OWDevice::read32(this OWDevice& self) noexcept
    {
        return std::uint32_t();
    }

    std::uint64_t OWDevice::read64(this OWDevice& self) noexcept
    {
        return std::uint64_t();
    }

    void OWDevice::reset(this OWDevice& self) noexcept
    {
        if (self.initialized) {
            static rmt_receive_config_t rx_config{};
            rx_config.signal_range_min_ns = RX_MIN_NS;
            rx_config.signal_range_max_ns = (TIMING_H + TIMING_I) * 1000;

            static rmt_transmit_config_t tx_config{};
            tx_config.flags.eot_level = BUS_RELEASED;

            ESP_ERROR_CHECK(rmt_receive(self.rx_channel, self.rx_buffer, sizeof(self.rx_buffer), &rx_config));
            ESP_ERROR_CHECK(
                rmt_transmit(self.tx_channel, self.copy_encoder, &SYMBOL_RESET, sizeof(SYMBOL_RESET), &tx_config));

            rmt_rx_done_event_data_t event;
            xQueueReceive(self.rx_queue, &event, pdMS_TO_TICKS(RMT_TIMEOUT_MS));

            ESP_ERROR_CHECK(rmt_tx_wait_all_done(self.tx_channel, RMT_TIMEOUT_MS));
        }
    }

    std::uint8_t OWDevice::parse_symbols(std::span<rmt_symbol_word_t> const symbols)
    {
        std::uint8_t ret = 0;
        std::uint8_t bit_count = 0;
        for (const auto& symbol : symbols) {
            if (bit_count < 8) {
                if (symbol.duration0 <= TIMING_A + RX_MARGIN_US &&
                    (symbol.duration1 == 0 || symbol.duration1 >= TIMING_E)) {
                    ret |= (1 << bit_count);
                    ++bit_count;
                } else if (symbol.duration0 >= TIMING_A + TIMING_E) {
                    ++bit_count;
                }
            } else {
                break;
            }
        }
        return ret;
    }

    void OWDevice::initialize_rx(this OWDevice& self, gpio_num_t const gpio_num) noexcept
    {
        rmt_rx_channel_config_t rx_channel_config{};
        rx_channel_config.gpio_num = gpio_num;
        rx_channel_config.clk_src = RMT_CLK_SRC_APB;
        rx_channel_config.resolution_hz = RESOLUTION_HZ;
        rx_channel_config.mem_block_symbols = RX_MEM_BLOCK_SYMBOLS;
        rx_channel_config.flags.invert_in = false;
        rx_channel_config.flags.with_dma = false;
        rx_channel_config.intr_priority = 1;

        rmt_rx_event_callbacks_t rx_callbacks{};
        rx_callbacks.on_recv_done = &rx_event_callback;

        self.rx_queue = xQueueCreateStatic(1, sizeof(rmt_rx_done_event_data_t), rx_queue_storage, &rx_queue_data);

        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_config, &self.rx_channel));
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(self.rx_channel, &rx_callbacks, self.rx_queue));
        ESP_ERROR_CHECK(rmt_enable(self.rx_channel));
    }

    void OWDevice::initialize_tx(this OWDevice& self, gpio_num_t const gpio_num) noexcept
    {
        rmt_tx_channel_config_t tx_channel_config;
        tx_channel_config.gpio_num = gpio_num;
        tx_channel_config.clk_src = RMT_CLK_SRC_APB;
        tx_channel_config.resolution_hz = RESOLUTION_HZ;
        tx_channel_config.mem_block_symbols = TX_MEM_BLOCK_SYMBOLS;
        tx_channel_config.trans_queue_depth = TX_QUEUE_DEPTH;
        tx_channel_config.flags.invert_out = true;
        tx_channel_config.flags.io_loop_back = true;
        tx_channel_config.flags.io_od_mode = true;
        tx_channel_config.intr_priority = 1;

        rmt_copy_encoder_config_t copy_encoder_config{};

        rmt_bytes_encoder_config_t bytes_encoder_config{};
        bytes_encoder_config.bit0 = SYMBOL_0;
        bytes_encoder_config.bit1 = SYMBOL_1;

        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_config, &self.tx_channel));
        ESP_ERROR_CHECK(rmt_enable(self.tx_channel));
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &self.copy_encoder));
        ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &self.bytes_encoder));
    }

    void OWDevice::deinitialize_rx(this OWDevice& self) noexcept
    {
        ESP_ERROR_CHECK(rmt_disable(self.rx_channel));
        ESP_ERROR_CHECK(rmt_del_channel(self.rx_channel));
    }

    void OWDevice::deinitialize_tx(this OWDevice& self) noexcept
    {
        ESP_ERROR_CHECK(rmt_disable(self.tx_channel));
        ESP_ERROR_CHECK(rmt_del_channel(self.tx_channel));
        ESP_ERROR_CHECK(rmt_del_encoder(self.copy_encoder));
        ESP_ERROR_CHECK(rmt_del_encoder(self.bytes_encoder));
    }

}; // namespace esp32_utility