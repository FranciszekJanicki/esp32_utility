#include "i2c_device.hpp"
#include <cstdio>

namespace esp32_utility {

    void I2CDevice::initialize(this I2CDevice const& self) noexcept
    {
        if (self.i2c_device != nullptr) {
            //  if (i2c_master_probe(self.i2c_device->master_bus,
            // self.i2c_device->device_address,
            //                      TIMEOUT) == ESP_OK) {
        } else {
            while (1) {
                std::puts("ERROR\n\r");
            }
        }
    }

    void
    I2CDevice::transmit(this I2CDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept
    {
        i2c_master_transmit(self.i2c_device, data, size, TIMEOUT);
    }

    void I2CDevice::receive(this I2CDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept
    {
        i2c_master_receive(self.i2c_device, data, size, TIMEOUT);
    }

    void I2CDevice::write(this I2CDevice const& self,
                          std::uint8_t const address,
                          std::uint8_t* const data,
                          std::size_t const size) noexcept
    {
        if (auto address_data = static_cast<std::uint8_t*>(std::malloc(size + 1UL))) {
            std::memcpy(address_data, &address, 1UL);
            std::memcpy(address_data + 1UL, address_data, size);
            i2c_master_transmit(self.i2c_device, address_data, size + 1UL, TIMEOUT);
            std::free(address_data);
        }
    }

    void I2CDevice::read(this I2CDevice const& self,
                         std::uint8_t const address,
                         std::uint8_t* const data,
                         std::size_t const size) noexcept
    {
        i2c_master_transmit_receive(self.i2c_device, &address, 1UL, data, size, TIMEOUT);
    }

}; // namespace esp32_utility
