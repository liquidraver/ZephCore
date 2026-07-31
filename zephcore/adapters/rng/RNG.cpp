/*
 * SPDX-License-Identifier: MIT
 * RNG::nextInt - shared implementation
 */

#include <mesh/RNG.h>

namespace mesh {

uint32_t RNG::nextInt(uint32_t _min, uint32_t _max)
{
	uint32_t num;
	random((uint8_t *)&num, sizeof(num));
	if (_max <= _min) {
		return _min;
	}
	return (num % (_max - _min)) + _min;
}

} /* namespace mesh */
