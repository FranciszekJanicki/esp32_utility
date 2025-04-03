#include "spi_device.hpp"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "utility.hpp"

namespace ESP32_Utility {

    SPIDevice::SPIDevice(spi_device_handle_t const spi_device,
                         gpio_num_t const chip_select,
                         std::size_t const dma_buffer_size) noexcept :
        dma_buffer_size_{dma_buffer_size}, spi_device_{spi_device}, chip_select_{chip_select}

    {
        this->initialize();
    }

    void SPIDevice::transmit_bytes_dma(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = size * 8;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.tx_buffer = this->dma_buffer_;

        std::memcpy(this->dma_buffer_, data, size);

        spi_transaction_t* result;
        gpio_set_level(this->chip_select_, 0);
        spi_device_queue_trans(this->spi_device_, &transaction, TIMEOUT);
        spi_device_get_trans_result(this->spi_device_, &result, TIMEOUT);
        gpio_set_level(this->chip_select_, 1);
    }

    void SPIDevice::transmit_byte_dma(std::uint8_t const data) const noexcept
    {
        this->transmit_bytes_dma(std::array<std::uint8_t, 1UL>{data});
    }

    void SPIDevice::receive_bytes_dma(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = size * 8;
        transaction.flags = 0;
        transaction.rx_buffer = this->dma_buffer_;
        spi_transaction_t* result{};

        gpio_set_level(this->chip_select_, 0);
        spi_device_queue_trans(this->spi_device_, &transaction, TIMEOUT);
        spi_device_get_trans_result(this->spi_device_, &result, TIMEOUT);
        gpio_set_level(this->chip_select_, 1);

        std::memcpy(data, transaction.rx_buffer, size);
    }

    std::uint8_t SPIDevice::receive_byte_dma() const noexcept
    {
        return this->receive_bytes_dma<1UL>()[0];
    }

    void SPIDevice::transmit_bytes(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data, size);

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);
    }

    void SPIDevice::transmit_byte(std::uint8_t const data) const noexcept
    {
        this->transmit_bytes(std::array<std::uint8_t, 1UL>{data});
    }

    void SPIDevice::receive_bytes(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);

        std::memcpy(data, transaction.rx_data, size);
    }

    std::uint8_t SPIDevice::receive_byte() const noexcept
    {
        return this->receive_bytes<1UL>()[0];
    }

    void SPIDevice::read_bytes_dma(std::uint8_t const address,
                                   std::uint8_t* const data,
                                   std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.flags = 0;
        transaction.addr = address_to_read_command(address);
        transaction.rx_buffer = this->dma_buffer_;

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);

        std::memcpy(data, transaction.rx_buffer, size);
    }

    std::uint8_t SPIDevice::read_byte_dma(std::uint8_t const address) const noexcept
    {
        return this->read_bytes_dma<1UL>(address)[0];
    }

    void SPIDevice::write_bytes_dma(std::uint8_t const address,
                                    std::uint8_t* const data,
                                    std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.flags = 0;
        transaction.addr = address_to_write_command(address);
        transaction.tx_buffer = this->dma_buffer_;

        std::memcpy(this->dma_buffer_, data, size);

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);
    }

    void SPIDevice::write_byte_dma(std::uint8_t const address, std::uint8_t const data) const noexcept
    {
        this->write_bytes_dma(address, std::array<std::uint8_t, 1UL>{data});
    }

    void
    SPIDevice::read_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 0;
        transaction.rxlength = 8 * size;
        transaction.addr = address_to_read_command(address);
        transaction.flags = SPI_TRANS_USE_RXDATA;

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);

        std::memcpy(data, transaction.rx_data, size);
    }

    std::uint8_t SPIDevice::read_byte(std::uint8_t const address) const noexcept
    {
        return this->read_bytes<1UL>(address)[0];
    }

    void
    SPIDevice::write_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept
    {
        spi_transaction_t transaction{};
        transaction.length = 8 * size;
        transaction.rxlength = 0;
        transaction.addr = address_to_write_command(address);
        transaction.flags = SPI_TRANS_USE_TXDATA;

        std::memcpy(transaction.tx_data, data, size);

        gpio_set_level(this->chip_select_, 0);
        spi_device_polling_transmit(this->spi_device_, &transaction);
        gpio_set_level(this->chip_select_, 1);
    }

    void SPIDevice::write_byte(std::uint8_t const address, std::uint8_t const data) const noexcept
    {
        this->write_bytes(address, std::array<std::uint8_t, 1UL>{data});
    }

    std::uint8_t SPIDevice::address_to_read_command(std::uint8_t const address) noexcept
    {
        return address & ~(1U << (std::bit_width(address) - 1U));
    }

    std::uint8_t SPIDevice::address_to_write_command(std::uint8_t const address) noexcept
    {
        return address | (1U << (std::bit_width(address) - 1U));
    }

    void SPIDevice::initialize() noexcept
    {
        if (this->spi_device_ != nullptr) {
            gpio_set_level(this->chip_select_, 1U);

            this->dma_buffer_ = static_cast<std::uint8_t*>(heap_caps_malloc(this->dma_buffer_size_, MALLOC_CAP_DMA));
        }
    }

    void SPIDevice::deinitialize() noexcept
    {
        if (this->spi_device_ != nullptr) {
            gpio_set_level(this->chip_select_, 0U);

            if (this->dma_buffer_) {
                heap_caps_free(this->dma_buffer_);
            }
        }
    }

}; // namespace ESP32_Utility