/*
 * SPDX-License-Identifier: MIT
 * The slice of Arduino's Print / Stream API that code ported verbatim from
 * upstream MeshCore uses, so it compiles unchanged.
 */

#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

class Print {
public:
	virtual ~Print() {}

	virtual size_t write(uint8_t c) = 0;
	virtual size_t write(const uint8_t *buffer, size_t size)
	{
		size_t n = 0;
		while (n < size && write(buffer[n])) {
			n++;
		}
		return n;
	}
	size_t write(const char *str) { return str ? write((const uint8_t *)str, strlen(str)) : 0; }

	size_t print(char c) { return write((uint8_t)c); }
	size_t print(const char *str) { return write(str); }
	size_t println(const char *str) { return write(str) + write("\r\n"); }

	/* Numbers, as Arduino prints them (ConfigSerializer). int32_t is long on
	 * some targets and int on others, so both widths are here. */
	size_t print(long n, int base = 10)
	{
		char buf[24];
		int len = (base == 16) ? snprintf(buf, sizeof(buf), "%lx", (unsigned long)n)
				       : snprintf(buf, sizeof(buf), "%ld", n);
		return write((const uint8_t *)buf, (size_t)len);
	}
	size_t print(unsigned long n, int base = 10)
	{
		char buf[24];
		int len = snprintf(buf, sizeof(buf), (base == 16) ? "%lx" : "%lu", n);
		return write((const uint8_t *)buf, (size_t)len);
	}
	size_t print(int n, int base = 10) { return print((long)n, base); }
	size_t print(unsigned int n, int base = 10) { return print((unsigned long)n, base); }

	/* Arduino's Print::printFloat, digit for digit, so a value is written as
	 * upstream would write it, without relying on libc float formatting. */
	size_t print(double number, int digits = 2)
	{
		if (number != number) {
			return print("nan");
		}
		if (number > 4294967040.0) {
			return print("ovf");
		}
		if (number < -4294967040.0) {
			return print("-ovf");
		}
		size_t n = 0;

		if (number < 0.0) {
			n += print('-');
			number = -number;
		}
		double rounding = 0.5;

		for (int i = 0; i < digits; ++i) {
			rounding /= 10.0;
		}
		number += rounding;
		unsigned long int_part = (unsigned long)number;
		double remainder = number - (double)int_part;

		n += print(int_part);
		if (digits > 0) {
			n += print('.');
		}
		while (digits-- > 0) {
			remainder *= 10.0;
			unsigned int to_print = (unsigned int)remainder;

			n += print(to_print);
			remainder -= to_print;
		}
		return n;
	}

	/* Longer output is truncated; the ported callers print short lines. */
	size_t printf(const char *format, ...) __attribute__((format(printf, 2, 3)))
	{
		char buf[160];
		va_list args;
		va_start(args, format);
		int n = vsnprintf(buf, sizeof(buf), format, args);
		va_end(args);
		if (n < 0) {
			return 0;
		}
		return write((const uint8_t *)buf, (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1);
	}
};

class Stream : public Print {
public:
	virtual int available() = 0;
	virtual int read() = 0;
	virtual int peek() = 0;
	virtual void flush() {}
};
