#include "spi_device.hpp"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "utility.hpp"

namespace Utility {

    SPIDevice::SPIDevice(spi_device_handle_t const spi_device,
                         gpio_num_t const chip_select,
                         std::size_t const dma_buffer_size) noexcept :
        dma_buffer_size_{dma_buffer_size}, spi_device_{spi_device}, chip_select_{chip_select}

    {
        this->initialize();
    }

    void SPIDevice::transmit_byte_dma(std::uint8_t const byte) const noexcept
    {
        this->transmit_bytes_dma(std::array<std::uint8_t, 1UL>{byte});
    }

    std::uint8_t SPIDevice::receive_byte_dma() const noexcept
    {
        return this->receive_bytes_dma<1UL>()[0];
    }

    void SPIDevice::transmit_byte(std::uint8_t const byte) const noexcept
    {
        this->transmit_bytes(std::array<std::uint8_t, 1UL>{byte});
    }

    std::uint8_t SPIDevice::receive_byte() const noexcept
    {
        return this->receive_bytes<1UL>()[0];
    }

    std::uint8_t SPIDevice::read_byte_dma(std::uint8_t const reg_address) const noexcept
    {
        return this->read_bytes_dma<1UL>(reg_address)[0];
    }

    void SPIDevice::write_byte_dma(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->write_bytes_dma(reg_address, std::array<std::uint8_t, 1UL>{byte});
    }

    std::uint8_t SPIDevice::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->read_bytes<1UL>(reg_address)[0];
    }

    void SPIDevice::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->write_bytes(reg_address, std::array<std::uint8_t, 1UL>{byte});
    }

    std::uint8_t SPIDevice::reg_address_to_read_command(std::uint8_t const reg_address) noexcept
    {
        return reg_address & ~(1U << (std::bit_width(reg_address) - 1U));
    }

    std::uint8_t SPIDevice::reg_address_to_write_command(std::uint8_t const reg_address) noexcept
    {
        return reg_address | (1U << (std::bit_width(reg_address) - 1U));
    }

    void SPIDevice::initialize() noexcept
    {
        if (this->spi_device_ != nullptr) {
            gpio_set_level(this->chip_select_, 1U);

            this->dma_buffer_ = static_cast<std::uint8_t*>(heap_caps_malloc(this->dma_buffer_size_, MALLOC_CAP_DMA));

            if (this->dma_buffer_) {
                this->initialized_ = true;
            }
        }
    }

    void SPIDevice::deinitialize() noexcept
    {
        if (this->spi_device_ != nullptr) {
            gpio_set_level(this->chip_select_, 0U);

            if (this->dma_buffer_) {
                heap_caps_free(this->dma_buffer_);
                this->initialized_ = false;
            }
        }
    }

}; // namespace Utility