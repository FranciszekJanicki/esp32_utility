#ifndef i2c_deviceHPP
#define i2c_deviceHPP

#include "driver/i2c_master.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <utility>

namespace esp32_utility {

    struct I2CDevice {
    public:
        void initialize(this I2CDevice const& self) noexcept;

        template <std::size_t SIZE>
        void transmit(this I2CDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept;
        void transmit(this I2CDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive(this I2CDevice const& self) noexcept;
        void receive(this I2CDevice const& self, std::uint8_t* const data, std::size_t const size) noexcept;

        template <std::size_t SIZE>
        void write(this I2CDevice const& self,
                   std::uint8_t const address,
                   std::array<std::uint8_t, SIZE> const& data) noexcept;
        void write(this I2CDevice const& self,
                   std::uint8_t const address,
                   std::uint8_t* const data,
                   std::size_t const size) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read(this I2CDevice const& self, std::uint8_t const address) noexcept;
        void read(this I2CDevice const& self,
                  std::uint8_t const address,
                  std::uint8_t* const data,
                  std::size_t const size) noexcept;

    private:
        static constexpr std::uint32_t TIMEOUT = 100UL;
        static constexpr std::uint32_t SCAN_RETRIES = 10UL;

        i2c_master_dev_handle_t i2c_device = nullptr;
    };

    template <std::size_t SIZE>
    inline void I2CDevice::transmit(this I2CDevice const& self, std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        i2c_master_transmit(self.i2c_device, data.data(), data.size(), TIMEOUT);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> I2CDevice::receive(this I2CDevice const& self) noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        i2c_master_receive(self.i2c_device, data.data(), data.size(), TIMEOUT);

        return data;
    }

    template <std::size_t SIZE>
    inline void I2CDevice::write(this I2CDevice const& self,
                                 std::uint8_t const address,
                                 std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        auto address_data = std::array<std::uint8_t, 1UL + SIZE>{};
        std::memcpy(address_data.data(), &address, 1UL);
        std::memcpy(address_data.data() + 1UL, address_data.data(), SIZE);

        i2c_master_transmit(self.i2c_device, address_data.data(), address_data.size(), TIMEOUT);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> I2CDevice::read(this I2CDevice const& self,
                                                          std::uint8_t const address) noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        i2c_master_transmit_receive(self.i2c_device, &address, 1UL, data.data(), data.size(), TIMEOUT);

        return data;
    }

}; // namespace esp32_utility

#endif // i2c_deviceHPP
