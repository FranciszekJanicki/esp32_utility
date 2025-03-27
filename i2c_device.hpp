#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "driver/i2c_master.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace Utility {

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
        void transmit_bytes(std::array<std::uint8_t, SIZE> const& bytes) const noexcept;
        void transmit_byte(std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive_bytes() const noexcept;
        std::uint8_t receive_byte() const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;
        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const reg_address, std::array<std::uint8_t, SIZE> const& bytes) const noexcept;
        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

    private:
        static constexpr std::uint32_t TIMEOUT{100U};
        static constexpr std::uint32_t SCAN_RETRIES{10U};

        void initialize() noexcept;

        bool initialized_{false};

        i2c_master_dev_handle_t i2c_device_{nullptr};
    };

    template <std::size_t SIZE>
    void I2CDevice::transmit_bytes(std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        std::array<std::uint8_t, SIZE> transmit{bytes};
        if (this->initialized_) {
            i2c_master_transmit(this->i2c_device_, transmit.data(), transmit.size(), TIMEOUT);
        }
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> I2CDevice::receive_bytes() const noexcept
    {
        std::array<std::uint8_t, SIZE> receive{};
        if (this->initialized_) {
            i2c_master_receive(this->i2c_device_, receive.data(), receive.size(), TIMEOUT);
        }
        return receive;
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> I2CDevice::read_bytes(std::uint8_t const reg_address) const noexcept
    {
        std::array<std::uint8_t, SIZE> read{};
        if (this->initialized_) {
            i2c_master_transmit_receive(this->i2c_device_,
                                        &reg_address,
                                        sizeof(reg_address),
                                        read.data(),
                                        read.size(),
                                        TIMEOUT);
        }
        return read;
    }

    template <std::size_t SIZE>
    void I2CDevice::write_bytes(std::uint8_t const reg_address,
                                std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        if (this->initialized_) {
            std::array<std::uint8_t, 1UL + SIZE> write{reg_address};
            std::memcpy(write.data() + 1UL, bytes.data(), SIZE);
            i2c_master_transmit(this->i2c_device_, write.data(), write.size(), TIMEOUT);
        }
    }

}; // namespace Utility

#endif // I2C_DEVICE_HPP
