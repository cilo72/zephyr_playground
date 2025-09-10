/*
  Copyright (c) 2023 Daniel Zwirner
  SPDX-License-Identifier: MIT-0
*/

#pragma once

#include <stdint.h>
#include <zephyr/drivers/spi.h>
namespace cilo72
{
    namespace hw
    {
        /**
         * @brief The SPIDevice class provides an interface to an SPI device.
         */
        class SPIDevice
        {
        public:
            /**
             * @brief Constructs an SPIDevice object with the given SPI bus, CSN pin, baudrate, data bits, clock polarity and clock phase.
             * @param spiBus The SPIBus object representing the SPI bus.
             * @param pin_spi_csn The CSN (chip select not) pin number.
             * @param baudrate The baudrate in Hz.
             * @param data_bits The number of data bits per transfer.
             * @param cpol The clock polarity.
             * @param cpha The clock phase.
             */
            SPIDevice(const struct spi_dt_spec & spi_dt_spec);

            /**
             * @brief Transfers data over the SPI bus.
             * @param tx The data to transmit.
             * @param rx The buffer to receive the data.
             * @param len The length of the data.
             */
            void xfer(uint8_t *tx, uint8_t *rx, size_t len) const;

            /**
             * @brief Writes data to the SPI bus.
             * @param tx The data to transmit.
             * @param len The length of the data.
             * @param repeat The number of times to repeat the data.
             */
            void write(const uint8_t *tx, size_t len, uint32_t repeat = 1) const;


        private:
            struct spi_dt_spec spi_dt_spec_;
        };
    }
}
