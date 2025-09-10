/*
  Copyright (c) 2023 Daniel Zwirner
  SPDX-License-Identifier: MIT-0
*/

#include "cilo72/hw/spi_device.h"

namespace cilo72
{
  namespace hw
  {
    SPIDevice::SPIDevice(const struct spi_dt_spec & spi_dt_spec)
        : spi_dt_spec_(spi_dt_spec)
    {
    }

    void SPIDevice::xfer(uint8_t *tx, uint8_t *rx, size_t len) const
    {
      struct spi_buf tx_buf      = {.buf = tx, .len = len};
      struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

      struct spi_buf rx_buf      = {.buf = rx, .len = len};
      struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

       
      spi_transceive_dt(&spi_dt_spec_, &tx_bufs, &rx_bufs);
    }

  }
}