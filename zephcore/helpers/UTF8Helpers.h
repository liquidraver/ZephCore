/*
 * SPDX-License-Identifier: MIT
 * ZephCore UTF8Helpers - UTF-8 aware truncation
 *
 * Ported from Arduino MeshCore 79dc1de6 ("fix: preserve UTF-8 advert names").
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace mesh {

inline bool isUtf8Continuation(uint8_t byte)
{
	return (byte & 0xC0) == 0x80;
}

/**
 * Length of the longest prefix of `text` that is (a) valid UTF-8 and (b) no
 * longer than `max_bytes`.  Never splits a multi-byte code point, and rejects
 * overlong encodings, surrogates and out-of-range 4-byte sequences.
 *
 * @returns byte count to copy; 0 if not even one code point fits
 */
inline size_t validUtf8PrefixLength(const char *text, size_t max_bytes)
{
	if (text == nullptr) return 0;

	size_t offset = 0;
	while (text[offset] != '\0') {
		const uint8_t first = (uint8_t)text[offset];
		size_t sequence_length = 0;

		if (first <= 0x7F) {
			sequence_length = 1;
		} else if (first >= 0xC2 && first <= 0xDF) {
			sequence_length = 2;
		} else if (first >= 0xE0 && first <= 0xEF) {
			sequence_length = 3;
		} else if (first >= 0xF0 && first <= 0xF4) {
			sequence_length = 4;
		} else {
			break;  /* continuation byte or overlong 2-byte lead */
		}

		if (offset + sequence_length > max_bytes) break;

		bool complete = true;
		for (size_t i = 1; i < sequence_length; i++) {
			if (text[offset + i] == '\0' ||
			    !isUtf8Continuation((uint8_t)text[offset + i])) {
				complete = false;
				break;
			}
		}
		if (!complete) break;

		/* Reject overlong 3-byte forms and UTF-16 surrogates. */
		if (sequence_length == 3) {
			const uint8_t second = (uint8_t)text[offset + 1];
			if ((first == 0xE0 && second < 0xA0) ||
			    (first == 0xED && second > 0x9F)) break;
		} else if (sequence_length == 4) {
			const uint8_t second = (uint8_t)text[offset + 1];
			if ((first == 0xF0 && second < 0x90) ||
			    (first == 0xF4 && second > 0x8F)) break;
		}

		offset += sequence_length;
	}
	return offset;
}

}  // namespace mesh
