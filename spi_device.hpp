#ifndef SPI_DEVICE_HPP
#define SPI_DEVICE_HPP

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <utility>

namespace esp32_utility {

    struct SPIDevice {
    public:
        void initialize(this SPIDevice& self, std::size_t const dma_buffer_size) noexcept;
        void deinitialize(this SPIDevice& self) noexcept;

        template <std::size_t SIZE>
        void transmit_dma(this SPIDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept;
        void transmit_dma(this SPIDevice const& self, std::uint8_t const* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive_dma(this SPIDevice const& self) noexcept;
        void receive_dma(this SPIDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        void transmit(this SPIDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept;
        void transmit(this SPIDevice const& self, std::uint8_t const* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive(this SPIDevice const& self) noexcept;
        void receive(this SPIDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_dma(this SPIDevice const& self, std::uint8_t const address) noexcept;
        void read_dma(this SPIDevice const& self,
                      std::uint8_t const address,
                      std::uint8_t* const data,
                      std::size_t const size) noexcept;

        template <std::size_t SIZE>
        void write_dma(this SPIDevice const& self,
                       std::uint8_t const address,
                       std::array<std::uint8_t, SIZE> const& data) noexcept;
        void write_dma(this SPIDevice const& self,
                       std::uint8_t const address,
                       std::uint8_t const* const data,
                       std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read(this SPIDevice const& self, std::uint8_t const address) noexcept;
        void read(this SPIDevice const& self,
                  std::uint8_t const address,
                  std::uint8_t* const data,
                  std::size_t const size) noexcept;

        template <std::size_t SIZE>
        void write(this SPIDevice const& self,
                   std::uint8_t const address,
                   std::array<std::uint8_t, SIZE> const& data) noexcept;
        void write(this SPIDevice const& self,
                   std::uint8_t const address,
                   std::uint8_t const* const data,
                   std::size_t const size) noexcept;

        std::uint8_t* dma_buffer = {};
        std::size_t dma_buffer_size = {};

        spi_device_handle_t spi_device = {};
        gpio_num_t chip_select = GPIO_NUM_NC;

    private:
        static std::uint8_t address_to_read_command(std::uint8_t const address) noexcept;
        static std::uint8_t address_to_write_command(std::uint8_t const address) noexcept;

        static constexpr std::uint32_t TIMEOUT = 100UL;
    };

    template <std::size_t SIZE>
    inline void SPIDevice::transmit_dma(this SPIDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = SIZE * 8;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.tx_buffer = self.dma_buffer;

        std::memcpy(self.dma_buffer, data.data(), data.size());

        spi_transaction_t* result;
        gpio_set_level(self.chip_select, 0);
        spi_device_queue_trans(self.spi_device, &transaction, TIMEOUT);
        spi_device_get_trans_result(self.spi_device, &result, TIMEOUT);
        gpio_set_level(self.chip_select, 1);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> SPIDevice::receive_dma(this SPIDevice const& self) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 0;
        transaction.rxlength = SIZE * 8;
        transaction.flags = 0;
        transaction.rx_buffer = self.dma_buffer;
        spi_transaction_t* result = {};

        gpio_set_level(self.chip_select, 0);
        spi_device_queue_trans(self.spi_device, &transaction, TIMEOUT);
        spi_device_get_trans_result(self.spi_device, &result, TIMEOUT);
        gpio_set_level(self.chip_select, 1);

        auto data = std::array<std::uint8_t, SIZE>{};
        std::memcpy(data.data(), transaction.rx_buffer, data.size());

        return data;
    }

    template <std::size_t SIZE>
    void SPIDevice::transmit(this SPIDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 8 * SIZE;
        transaction.rxlength = 0;
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data.data(), data.size());

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> SPIDevice::receive(this SPIDevice const& self) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 0;
        transaction.rxlength = 8 * SIZE;
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        auto data = std::array<std::uint8_t, SIZE>{};
        std::memcpy(data.data(), transaction.rx_data, data.size());

        return data;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> SPIDevice::read_dma(this SPIDevice const& self,
                                                              std::uint8_t const address) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 0;
        transaction.rxlength = 8 * SIZE;
        transaction.flags = 0;
        transaction.addr = address_to_read_command(address);
        transaction.rx_buffer = self.dma_buffer;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        auto data = std::array<std::uint8_t, SIZE>{};
        std::memcpy(data.data(), transaction.rx_buffer, data.size());

        return data;
    }

    template <std::size_t SIZE>
    inline void SPIDevice::write_dma(this SPIDevice const& self,
                                     std::uint8_t const address,
                                     std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 8 * SIZE;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.addr = address_to_write_command(address);
        transaction.tx_buffer = self.dma_buffer;

        std::memcpy(self.dma_buffer, data.data(), data.size());

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> SPIDevice::read(this SPIDevice const& self, std::uint8_t const address) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 0;
        transaction.rxlength = 8 * SIZE;
        transaction.addr = address_to_read_command(address);
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        auto data = std::array<std::uint8_t, SIZE>{};
        std::memcpy(data.data(), transaction.rx_data, data.size());

        return data;
    }

    template <std::size_t SIZE>
    void SPIDevice::write(this SPIDevice const& self,
                          std::uint8_t const address,
                          std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        spi_transaction_t transaction = {};
        transaction.length = 8 * SIZE;
        transaction.rxlength = 0;
        transaction.addr = address_to_write_command(address);
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data.data(), data.size());

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

}; // namespace esp32_utility

#endif // SPI_DEVICE_HPP