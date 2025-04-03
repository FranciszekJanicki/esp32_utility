#include "i2c_device.hpp"
#include <cstdio>

namespace ESP32_Utility {

    I2CDevice::I2CDevice(i2c_master_dev_handle_t const i2c_device) noexcept : i2c_device_{i2c_device}
    {
        this->initialize();
    }

    void I2CDevice::transmit_bytes(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        i2c_master_transmit(this->i2c_device_, data, size, TIMEOUT);
    }

    void I2CDevice::transmit_byte(std::uint8_t const data) const noexcept
    {
        this->transmit_bytes(std::array<std::uint8_t, 1UL>{data});
    }

    void I2CDevice::receive_bytes(std::uint8_t* const data, std::size_t const size) const noexcept
    {
        i2c_master_receive(this->i2c_device_, data, size, TIMEOUT);
    }

    std::uint8_t I2CDevice::receive_byte() const noexcept
    {
        return this->receive_bytes<1UL>()[0];
    }

    void
    I2CDevice::write_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept
    {
        if (auto address_data = static_cast<std::uint8_t*>(std::malloc(size + 1UL))) {
            std::memcpy(address_data, &address, 1UL);
            std::memcpy(address_data + 1UL, address_data, size);
            i2c_master_transmit(this->i2c_device_, address_data, size + 1UL, TIMEOUT);
            std::free(address_data);
        }
    }

    void I2CDevice::write_byte(std::uint8_t const address, std::uint8_t const data) const noexcept
    {
        this->write_bytes(address, std::array<std::uint8_t, 1UL>{data});
    }

    void
    I2CDevice::read_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept
    {
        i2c_master_transmit_receive(this->i2c_device_, &address, 1UL, data, size, TIMEOUT);
    }

    std::uint8_t I2CDevice::read_byte(std::uint8_t const address) const noexcept
    {
        return this->read_bytes<1UL>(address)[0];
    }

    void I2CDevice::initialize() noexcept
    {
        if (this->i2c_device_ != nullptr) {
            //  if (i2c_master_probe(this->i2c_device_->master_bus,
            // this->i2c_device_->device_address,
            //                      TIMEOUT) == ESP_OK) {
        } else {
            while (1) {
                std::puts("ERROR\n\r");
            }
        }
    }

}; // namespace ESP32_Utility
