/*
 * SPDX-License-Identifier: MIT
 * RateLimiter - simple rate limiting for repeaters
 */

#pragma once

#include <cstdint>

class RateLimiter {
    /* Members ordered to match constructor initialization order */
    uint16_t _maximum;
    uint32_t _secs;
    uint32_t _start_timestamp;
    uint16_t _count;

public:
    RateLimiter(uint16_t maximum, uint32_t secs)
        : _maximum(maximum), _secs(secs), _start_timestamp(0), _count(0) {}

    bool allow(uint32_t now) {
        if (now < _start_timestamp + _secs) {
            _count++;
            if (_count > _maximum) return false;  // deny
        } else {
            // time window now expired
            _start_timestamp = now;
            _count = 1;
        }
        return true;
    }

    void reset() {
        _start_timestamp = 0;
        _count = 0;
    }
};
