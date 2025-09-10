/*
  Copyright (c) 2023 Daniel Zwirner
  SPDX-License-Identifier: MIT-0
 */

#ifndef APP_DRIVERS_TMC5XXX_H_
#define APP_DRIVERS_TMC5XXX_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

__subsystem struct tmc5xxx_driver_api {
	int (*reg_read)(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val, uint8_t * status);
	int (*reg_write)(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val, uint8_t * status);
	void (*enable)(const struct device *dev, uint8_t enable);
};

__syscall int tmc5xxx_reg_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val, uint8_t * status);
__syscall int tmc5xxx_reg_write(const struct device *dev, const uint8_t reg_addr, uint32_t reg_val, uint8_t * status);
__syscall void tmc5xxx_enable(const struct device *dev, uint8_t enable);

static inline int z_impl_tmc5xxx_reg_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val, uint8_t * status)
{
	__ASSERT_NO_MSG(DEVICE_API_IS(tmc5xxx, dev));

	return DEVICE_API_GET(tmc5xxx, dev)->reg_read(dev, reg_addr, reg_val, status);
}

static inline int z_impl_tmc5xxx_reg_write(const struct device *dev, const uint8_t reg_addr, uint32_t reg_val, uint8_t * status)
{
	__ASSERT_NO_MSG(DEVICE_API_IS(tmc5xxx, dev));

	return DEVICE_API_GET(tmc5xxx, dev)->reg_write(dev, reg_addr, reg_val, status);
}

static inline void z_impl_tmc5xxx_enable(const struct device *dev, uint8_t enable)
{
	__ASSERT_NO_MSG(DEVICE_API_IS(tmc5xxx, dev));

	DEVICE_API_GET(tmc5xxx, dev)->enable(dev, enable);
}

#include <syscalls/tmc5xxx.h>
/** @} */

/** @} */

#ifdef __cplusplus
}
#endif


#endif /* APP_DRIVERS_TMC5XXX_H_ */
