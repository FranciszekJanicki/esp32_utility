#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "driver/i2c_master.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <utility>

namespace ESP32_Utility {

    struct I2CDevice {
    public:
        I2CDevice() noexcept = default;
        I2CDevice(i2c_master_dev_handle_t const i2c_device) noexcept;

        I2CDevice(I2CDevice const& other) = delete;
        I2CDevice(I2CDevice&& other) noexcept = default;

        I2CDevice& operator=(I2CDevice const& other) = delete;
        I2CDevice& operator=(I2CDevice&& other) noexcept = default;

        ~I2CDevice() noexcept = default;

        template <std::size_t SIZE>
        void transmit_bytes(std::array<std::uint8_t, SIZE> const& data) const noexcept;
        void transmit_bytes(std::uint8_t* const data, std::size_t const size) const noexcept;
        void transmit_byte(std::uint8_t const data) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive_bytes() const noexcept;
        void receive_bytes(std::uint8_t* const data, std::size_t const size) const noexcept;
        std::uint8_t receive_byte() const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const address, std::array<std::uint8_t, SIZE> const& data) const noexcept;
        void write_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept;
        void write_byte(std::uint8_t const address, std::uint8_t const data) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const address) const noexcept;
        void read_bytes(std::uint8_t const address, std::uint8_t* const data, std::size_t const size) const noexcept;
        std::uint8_t read_byte(std::uint8_t const address) const noexcept;

    private:
        static constexpr std::uint32_t TIMEOUT{100U};
        static constexpr std::uint32_t SCAN_RETRIES{10U};

        void initialize() noexcept;

        i2c_master_dev_handle_t i2c_device_{nullptr};
    };

    template <std::size_t SIZE>
    inline void I2CDevice::transmit_bytes(std::array<std::uint8_t, SIZE> const& data) const noexcept
    {
        i2c_master_transmit(this->i2c_device_, data.data(), data.size(), TIMEOUT);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> I2CDevice::receive_bytes() const noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        i2c_master_receive(this->i2c_device_, data.data(), data.size(), TIMEOUT);

        return data;
    }

    template <std::size_t SIZE>
    inline void I2CDevice::write_bytes(std::uint8_t const address,
                                       std::array<std::uint8_t, SIZE> const& data) const noexcept
    {
        auto address_data = std::array<std::uint8_t, 1UL + SIZE>{};
        std::memcpy(address_data.data(), &address, 1UL);
        std::memcpy(address_data.data() + 1UL, address_data.data(), SIZE);

        i2c_master_transmit(this->i2c_device_, address_data.data(), address_data.size(), TIMEOUT);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> I2CDevice::read_bytes(std::uint8_t const address) const noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        i2c_master_transmit_receive(this->i2c_device_, &address, 1UL, data.data(), data.size(), TIMEOUT);

        return data;
    }

}; // namespace ESP32_Utility

#endif // I2C_DEVICE_HPP
