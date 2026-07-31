/*
 * SPDX-License-Identifier: MIT
 * ZephCore RNG interface - matches Utils.h
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/* On STM32, CMSIS defines an object-like macro `RNG` that collides with the
 * mesh::RNG class below. The fixup header neutralizes it order-independently. */
#include <mesh/stm32_cmsis_fixup.h>

namespace mesh {

class RNG {
public:
	virtual void random(uint8_t *dest, size_t sz) = 0;

	uint32_t nextInt(uint32_t _min, uint32_t _max);
};

} /* namespace mesh */
