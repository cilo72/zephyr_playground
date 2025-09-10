/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tmc5xxx_spi

#include <zephyr/device.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tmc5xxx.h>

LOG_MODULE_REGISTER(tmc5xxx_spi, CONFIG_TMC5XXX_LOG_LEVEL);

#define TMC5XXX_WRITE_BIT        0x80U
#define TMC5XXX_ADDRESS_MASK     0x7FU


/* STEP 3.1 Define data structure */
struct tmc5xxx_spi_data 
{
	uint8_t last_read_addr;
	struct gpio_callback diag0_cb;
};

/* STEP 3.2 Define configuration structure */
struct tmc5xxx_spi_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec diag0_gpio;
	struct gpio_dt_spec enable_gpio;
};

#define BUFFER_SIZE 5U

static int tmc51xx_spi_reg_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val, uint8_t * status)
{
	const struct tmc5xxx_spi_config *config = dev->config;
	struct tmc5xxx_spi_data *data = dev->data;

	uint8_t tx_buffer[BUFFER_SIZE] = {TMC5XXX_WRITE_BIT | reg_addr, reg_val >> 24, reg_val >> 16,
					  reg_val >> 8, reg_val};
	uint8_t rx_buffer[BUFFER_SIZE];
	int ret;

	const struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer,
		.len = sizeof(tx_buffer),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);

	data->last_read_addr = 0xFF;

	if (ret < 0) {
		return ret;
	}

	*status = rx_buffer[0];

	return ret;	
}

static int tmc51xx_spi_reg_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val, uint8_t * status)
{
	const struct tmc5xxx_spi_config *config = dev->config;
	struct tmc5xxx_spi_data *data = dev->data;

	uint8_t tx_buffer[BUFFER_SIZE] = {TMC5XXX_ADDRESS_MASK & reg_addr, 0U, 0U, 0U, 0U};
	uint8_t rx_buffer[BUFFER_SIZE];
	int ret;

	const struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer,
		.len = sizeof(tx_buffer),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	/** send read with the address byte */
	if (data->last_read_addr != reg_addr)
	{
		ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
		data->last_read_addr = reg_addr;
		if (ret < 0) {
			return ret;
		}
	}

	/** read the value from the address */
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret < 0) {
		return ret;
	}

	*status = rx_buffer[0];

	*reg_val = ((uint32_t)rx_buffer[1] << 24) + ((uint32_t)rx_buffer[2] << 16) +
		((uint32_t)rx_buffer[3] << 8) + (uint32_t)rx_buffer[4];

	return ret;
}

static void tmc51xx_spi_enable(const struct device *dev, uint8_t enable)
{
	const struct tmc5xxx_spi_config *config = dev->config;
	gpio_pin_set_dt(&config->enable_gpio, enable);
}

static DEVICE_API(tmc5xxx, tmc5xxx_spi_api) = {
    .reg_read = &tmc51xx_spi_reg_read,
	.reg_write = &tmc51xx_spi_reg_write,
	.enable = &tmc51xx_spi_enable
};

static void __maybe_unused tmc5xxx_diag0_gpio_callback_handler(const struct device *port,
							       struct gpio_callback *cb,
							       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	//struct tmc5xxx_spi_data *stepper_data = CONTAINER_OF(cb, struct tmc5xxx_spi_data, diag0_cb);

	//k_work_reschedule(&stepper_data->rampstat_callback_dwork, K_NO_WAIT);
}

static int tmc5xxx_spi_init(const struct device *dev)
{
    const struct tmc5xxx_spi_config *config = dev->config;
    struct tmc5xxx_spi_data *data = dev->data;
	
	data->last_read_addr = 0xFF;

	if(!spi_is_ready_dt(&config->spi))
	{
        LOG_ERR("SPI not ready");
        return -ENODEV;
	}

	if(config->enable_gpio.port)
	{
    	if (!gpio_is_ready_dt(&config->enable_gpio)) {
    	    LOG_ERR("Enable GPIO not ready");
    	    return -ENODEV;
    	}

		int err;
		err = gpio_pin_configure_dt(&config->enable_gpio, 	GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			LOG_ERR("Could not configure Enable GPIO (%d)", err);
			return err;
		}
	}

	if(config->diag0_gpio.port)
	{
    	if (!gpio_is_ready_dt(&config->diag0_gpio)) {
    	    LOG_ERR("LED GPIO not ready");
    	    return -ENODEV;
    	}

		int err;
		err = gpio_pin_configure_dt(&config->diag0_gpio, GPIO_INPUT);
		if (err < 0) {
			LOG_ERR("Could not configure DIAG0 GPIO (%d)", err);
			return err;
		}

		err = gpio_pin_interrupt_configure_dt(&config->diag0_gpio, GPIO_INT_EDGE_RISING);
		if (err) {
			LOG_ERR("failed to configure DIAG0 interrupt (err %d)", err);
			return -EIO;
		}

		/* Initialize and add GPIO callback */
		gpio_init_callback(&data->diag0_cb, tmc5xxx_diag0_gpio_callback_handler, BIT(config->diag0_gpio.pin));

		err = gpio_add_callback(config->diag0_gpio.port, &data->diag0_cb);
		if (err < 0) {
			LOG_ERR("Could not add DIAG0 pin GPIO callback (%d)", err);
			return -EIO;
		}

	}

    return 0;
}

#define TMC5XXX_SPI_DEFINE(inst)                                               \
    static struct tmc5xxx_spi_data data##inst;                                 \
                                                                               \
    static const struct tmc5xxx_spi_config config##inst = {                    \
        .spi = SPI_DT_SPEC_INST_GET(inst,                                      \
					(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL |   \
					 SPI_MODE_CPHA | SPI_WORD_SET(8)),                         \
					0),                                                        \
        .diag0_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, diag0_gpios, {0}),        \
        .enable_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),      \
    };                                                                         \
                                                                               \
    DEVICE_DT_INST_DEFINE(inst, tmc5xxx_spi_init, NULL, &data##inst,           \
                  &config##inst, POST_KERNEL,                                  \
                  CONFIG_TMC5XXX_INIT_PRIORITY,                                \
                  &tmc5xxx_spi_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5XXX_SPI_DEFINE)
