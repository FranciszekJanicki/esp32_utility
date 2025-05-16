#ifndef OW_DEVICE_HPP
#define OW_DEVICE_HPP

#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/rmt_types.h"
#include "soc/clk_tree_defs.h"
#include <cstdint>
#include <span>

namespace esp32_utility {

    struct OWDevice {
    public:
        void initialize(this OWDevice& self, gpio_num_t const gpio_num) noexcept;
        void deinitialize(this OWDevice& self) noexcept;

        void reset(this OWDevice& self) noexcept;

        void write8(this OWDevice& self, std::uint8_t const data) noexcept;
        void write16(this OWDevice& self, std::uint16_t const data) noexcept;
        void write32(this OWDevice& self, std::uint32_t const data) noexcept;
        void write64(this OWDevice& self, std::uint64_t const data) noexcept;

        std::uint8_t read8(this OWDevice& self) noexcept;
        std::uint16_t read16(this OWDevice& self) noexcept;
        std::uint32_t read32(this OWDevice& self) noexcept;
        std::uint64_t read64(this OWDevice& self) noexcept;

    private:
        static std::uint8_t parse_symbols(std::span<rmt_symbol_word_t> const symbols) noexcept;

        enum Timing : std::uint16_t {
            TIMING_A = 6,
            TIMING_B = 64,
            TIMING_C = 60,
            TIMING_D = 10,
            TIMING_E = 9,
            TIMING_F = 55,
            TIMING_G = 0,
            TIMING_H = 480,
            TIMING_I = 70,
            TIMING_J = 410,
        };

        static constexpr std::uint16_t BUS_ASSERTED = 1;
        static constexpr std::uint16_t BUS_RELEASED = 0;
        static constexpr std::uint32_t RESOLUTION_HZ = 1000000;
        static constexpr std::uint16_t TX_MEM_BLOCK_SYMBOLS = 64;
        static constexpr std::uint16_t TX_QUEUE_DEPTH = 4;
        static constexpr std::uint16_t MAX_READ_BITS = 64;
        static constexpr std::uint16_t RX_MEM_BLOCK_SYMBOLS = (MAX_READ_BITS + 2);
        static constexpr std::uint16_t RX_MIN_NS = 1000;
        static constexpr std::uint16_t RMT_TIMEOUT_MS = 1000;
        static constexpr std::uint16_t RX_MARGIN_US = 3;
        static constexpr std::uint16_t RX_BUFFER_SIZE = MAX_READ_BITS;

        static constexpr rmt_symbol_word_t SYMBOL_0 = {
            .duration0 = TIMING_C,
            .level0 = BUS_ASSERTED,
            .duration1 = TIMING_D,
            .level1 = BUS_RELEASED,
        };

        static constexpr rmt_symbol_word_t SYMBOL_1 = {
            .duration0 = TIMING_A,
            .level0 = BUS_ASSERTED,
            .duration1 = TIMING_B,
            .level1 = BUS_RELEASED,
        };

        static constexpr rmt_symbol_word_t SYMBOL_RESET = {
            .duration0 = TIMING_H,
            .level0 = BUS_ASSERTED,
            .duration1 = TIMING_I + TIMING_J,
            .level1 = BUS_RELEASED,
        };

        void initialize_rx(this OWDevice& self, gpio_num_t const gpio_num) noexcept;
        void initialize_tx(this OWDevice& self, gpio_num_t const gpio_num) noexcept;

        void deinitialize_rx(this OWDevice& self) noexcept;
        void deinitialize_tx(this OWDevice& self) noexcept;

        bool initialized = {};

        QueueHandle_t rx_queue = {};

        rmt_channel_handle_t tx_channel = {};
        rmt_channel_handle_t rx_channel = {};
        rmt_encoder_handle_t copy_encoder = {};
        rmt_encoder_handle_t bytes_encoder = {};

        rmt_symbol_word_t rx_buffer[RX_BUFFER_SIZE] = {};
    };

}; // namespace esp32_utility

#endif // OW_DEVICE_HPP
