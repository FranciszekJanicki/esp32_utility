#include "i2c_device.hpp"

namespace Utility {

    I2CDevice::I2CDevice(i2c_master_dev_handle_t const i2c_device) noexcept : i2c_device_{i2c_device}
    {
        this->initialize();
    }

    void I2CDevice::transmit_byte(std::uint8_t const byte) const noexcept
    {
        this->transmit_bytes(std::array<std::uint8_t, 1UL>{byte});
    }

    std::uint8_t I2CDevice::receive_byte() const noexcept
    {
        return this->receive_bytes<1UL>()[0];
    }

    std::uint8_t I2CDevice::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->read_bytes<1UL>(reg_address)[0];
    }

    void I2CDevice::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->write_bytes(reg_address, std::array<std::uint8_t, 1UL>{byte});
    }

    void I2CDevice::initialize() noexcept
    {
        if (this->i2c_device_ != nullptr) {
            // if (i2c_master_probe(this->i2c_device_->master_bus, this->i2c_device_->device_address, TIMEOUT) ==
            // ESP_OK) {
            this->initialized_ = true;
            //}
        }
    }

}; // namespace Utility
