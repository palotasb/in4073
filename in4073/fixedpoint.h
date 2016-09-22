#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <inttypes.h>

// Fixedpoint unsigned numbers of different widths.
typedef uint32_t        uq32_t;
typedef uint16_t        uq16_t;
typedef uint8_t         uq8_t;

typedef int32_t         q32_t;
typedef int16_t         q16_t;
typedef int8_t          q8_t;

#define ufp32_t(frac)   uq32_t
#define ufp16_t(frac)   uq16_t
#define ufp8_t(frac)    uq8_t

#define fp32_t(frac)    q32_t
#define fp16_t(frac)    q16_t
#define fp8_t(frac)     q8_t

#define f16p16_t        fp32_t(16)
#define f8p8_t          fp16_t(16)

// UTILITY DEFINITIONS
// ===================

// FP_FP_INT(i, frac) -- Create generic f_.frac fixedpoint number
// with integer part i and fractional part 0
#define FP_INT(i, frac)     ((i) << (frac))

// FP_XP(a, b, frac) -- Create generic f_.frac fixedpoint number
// with integer part a and fractional part (b / (2 ** frac))
#define FP_INT_FRAC(a, b, frac)     (((a) << (frac)) | (b))

// INT_FP(fp, frac) -- Get integer part of f_.frac fixedpoint number
#define INT_FP(fp, frac)     ((fp) >> (frac))

// FRAC_FP(i, frac) -- Get fractional part of f_.frac fixedpoint number
// 0b000111111 = (1 << (frac)) - 1
//      ^#frac
#define FRAC_FP(i, frac)     ((i) & ((1ul << (frac)) - 1))

// FLOAT_FP(fp, frac) -- Convert f_.frac format fixedpoint fp to float
#define FLOAT_FP(fp, frac)     ((float)(fp) / (1ul << (frac)))

// FP_FLOAT(f, frac) -- Convert float f to f_.frac format fixedpoint
#define FP_FLOAT(f, frac)     ((f) * (1ul << (frac)))

#define MUL_FP3(fpa, fpb, shra, shrb, shrr)      ((((fpa) >> (shra)) * ((fpb) >> (shrb))) >> (shrr))

#define MUL_FP_PRESH(fpa, fpb, shra, shrb)       MUL_FP3((fpa), (fpb), (shra), (shrb), 0)

#define MUL_FP_POSTSH(fpa, fpb, shrr)     MUL_FP3((fpa), (fpb), 0, 0, (shrr))

#define FP_EXTEND(fp, fraca, fracb)     ((fp) << ((fraca) - (fracb)))

#define FP_CHUNK(fp, fraca, fracb)     ((fp) >> ((fracb) - (fraca)))

#ifdef QUADCOPTER
    // Function to to integer square root
    uint32_t fp_sqrt(uint32_t n);
#endif // QUADCOPTER

#endif // FIXEDPOINT_H
