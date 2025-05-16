#include "spi_device.hpp"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

namespace esp32_utility {

    void SPIDevice::initialize(this SPIDevice& self, std::size_t const dma_buffer_size) noexcept
    {
        if (self.spi_device != nullptr) {
            gpio_set_level(self.chip_select, 1U);

            self.dma_buffer_size = dma_buffer_size;
            self.dma_buffer = static_cast<std::uint8_t*>(heap_caps_malloc(self.dma_buffer_size, MALLOC_CAP_DMA));
        }
    }

    void SPIDevice::deinitialize(this SPIDevice& self) noexcept
    {
        if (self.spi_device != nullptr) {
            gpio_set_level(self.chip_select, 0U);

            if (self.dma_buffer) {
                self.dma_buffer_size = 0UL;
                heap_caps_free(self.dma_buffer);
            }
        }
    }

    void
    SPIDevice::transmit_dma(this SPIDevice const& self, std::uint8_t const* const data, std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = size * 8;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.tx_buffer = self.dma_buffer;

        std::memcpy(self.dma_buffer, data, size);

        spi_transaction_t* result;
        gpio_set_level(self.chip_select, 0);
        spi_device_queue_trans(self.spi_device, &transaction, TIMEOUT);
        spi_device_get_trans_result(self.spi_device, &result, TIMEOUT);
        gpio_set_level(self.chip_select, 1);
    }

    void SPIDevice::receive_dma(this SPIDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = size * 8;
        transaction.flags = 0;
        transaction.rx_buffer = self.dma_buffer;
        spi_transaction_t* result{};

        gpio_set_level(self.chip_select, 0);
        spi_device_queue_trans(self.spi_device, &transaction, TIMEOUT);
        spi_device_get_trans_result(self.spi_device, &result, TIMEOUT);
        gpio_set_level(self.chip_select, 1);

        std::memcpy(data, transaction.rx_buffer, size);
    }

    void
    SPIDevice::transmit(this SPIDevice const& self, std::uint8_t const* const data, std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data, size);

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

    void SPIDevice::receive(this SPIDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        std::memcpy(data, transaction.rx_data, size);
    }

    void SPIDevice::read_dma(this SPIDevice const& self,
                             std::uint8_t const address,
                             std::uint8_t* const data,
                             std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.flags = 0;
        transaction.addr = address_to_read_command(address);
        transaction.rx_buffer = self.dma_buffer;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        std::memcpy(data, transaction.rx_buffer, size);
    }

    void SPIDevice::write_dma(this SPIDevice const& self,
                              std::uint8_t const address,
                              std::uint8_t const* const data,
                              std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.addr = address_to_write_command(address);
        transaction.tx_buffer = self.dma_buffer;

        std::memcpy(self.dma_buffer, data, size);

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

    void SPIDevice::read(this SPIDevice const& self,
                         std::uint8_t const address,
                         std::uint8_t* const data,
                         std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.addr = address_to_read_command(address);
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);

        std::memcpy(data, transaction.rx_data, size);
    }

    void SPIDevice::write(this SPIDevice const& self,
                          std::uint8_t const address,
                          std::uint8_t const* const data,
                          std::size_t const size) noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.addr = address_to_write_command(address);
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data, size);

        gpio_set_level(self.chip_select, 0);
        spi_device_polling_transmit(self.spi_device, &transaction);
        gpio_set_level(self.chip_select, 1);
    }

    std::uint8_t SPIDevice::address_to_read_command(std::uint8_t const address) noexcept
    {
        return address & ~(1U << (std::bit_width(address) - 1U));
    }

    std::uint8_t SPIDevice::address_to_write_command(std::uint8_t const address) noexcept
    {
        return address | (1U << (std::bit_width(address) - 1U));
    }

}; // namespace esp32_utility