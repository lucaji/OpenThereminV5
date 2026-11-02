// V5 firmware development (2026): Luca Cipressi (milletgrain).

#ifndef AUDIO_ARITHMETIC_H
#define AUDIO_ARITHMETIC_H

#include <stdint.h>

// These helpers preserve mathematical floor, including negative products.
// The portable paths are deliberately independent references for host tests.
static inline __attribute__((always_inline)) int16_t interpolateDelta(int16_t delta, uint8_t fraction) {
#ifdef __AVR__
    int16_t highProduct;
    uint8_t lowHigh;
    const uint8_t low = static_cast<uint8_t>(delta);
    const int8_t high = static_cast<int8_t>(static_cast<uint16_t>(delta) >> 8);
    // delta = 256 * signedHigh + unsignedLow. Only the upper byte of
    // unsignedLow * fraction contributes. Explicit byte operations avoid
    // AVR GCC expanding each promoted 8-bit product into two multiplications.
    asm("mul %2, %3\n\t"
        "mov %1, r1\n\t"
        "mulsu %4, %3\n\t"
        "movw %A0, r0\n\t"
        "clr __zero_reg__"
        : "=&r"(highProduct), "=&r"(lowHigh)
        : "r"(low), "a"(fraction), "a"(high)
        : "r0", "r1", "cc");
    return highProduct + lowHigh;
#else
    const int32_t product = static_cast<int32_t>(delta) * fraction;
    return product / 256 - (product < 0 && product % 256 != 0);
#endif
}

static inline __attribute__((always_inline)) int16_t scaleWaveExact(int16_t sample, uint16_t gain) {
#ifdef __AVR__
    uint16_t result;
    uint8_t middle;
    const uint8_t zero = 0;
    // Four unsigned byte products, retaining every carry into the upper word.
    // Correct the upper word for a negative sample; no rounded/low-gain bits
    // are discarded prematurely. Early-clobber outputs cannot overlap inputs.
    asm("mul %A2, %A3\n\t"
        "mov %1, r1\n\t"
        "mul %B2, %B3\n\t"
        "movw %A0, r0\n\t"
        "mul %A2, %B3\n\t"
        "add %1, r0\n\t"
        "adc %A0, r1\n\t"
        "adc %B0, %4\n\t"
        "mul %B2, %A3\n\t"
        "add %1, r0\n\t"
        "adc %A0, r1\n\t"
        "adc %B0, %4\n\t"
        "clr __zero_reg__\n\t"
        "sbrs %B2, 7\n\t"
        "rjmp 1f\n\t"
        "sub %A0, %A3\n\t"
        "sbc %B0, %B3\n\t"
        "1:"
        : "=&r"(result), "=&r"(middle)
        : "r"(sample), "r"(gain), "r"(zero)
        : "r0", "r1", "cc");
    return static_cast<int16_t>(result);
#else
    const int32_t product = static_cast<int32_t>(sample) * gain;
    return product / 65536 - (product < 0 && product % 65536 != 0);
#endif
}

#endif
