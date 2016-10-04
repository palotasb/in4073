#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <inttypes.h>

// Fixedpoint unsigned numbers of different widths.

// Fixedpoint unsigned 32 bit datatype.
typedef uint32_t        uq32_t;
// Fixedpoint unsigned 16 bit datatype.
typedef uint16_t        uq16_t;
// Fixedpoint unsigned 8 bit datatype.
typedef uint8_t         uq8_t;

// Fixedpoint signed 32 bit datatype.
typedef int32_t         q32_t;
// Fixedpoint signed 16 bit datatype.
typedef int16_t         q16_t;
// Fixedpoint signed 8 bit datatype.
typedef int8_t          q8_t;

// Fixedpoint 32 bit unsigned datatype. Fractional bits can be stated
// in the `frac` parameter.
#define ufp32_t(frac)   uq32_t
// Fixedpoint 16 bit unsigned datatype. Fractional bits can be stated
// in the `frac` parameter.
#define ufp16_t(frac)   uq16_t
// Fixedpoint 8 bit unsigned datatype. Fractional bits can be stated
// in the `frac` parameter.
#define ufp8_t(frac)    uq8_t

// Fixedpoint 32 bit signed datatype. Fractional bits can be stated in
// the `frac` parameter.
#define fp32_t(frac)    q32_t
// Fixedpoint 16 bit signed datatype. Fractional bits can be stated in
// the `frac` parameter.
#define fp16_t(frac)    q16_t
// Fixedpoint 8 bit signed datatype. Fractional bits can be stated in
// the `frac` parameter.
#define fp8_t(frac)     q8_t

#define f16p16_t        fp32_t(16)
#define f24p8_t         fp32_t(8)
#define f8p8_t          fp16_t(16)

// UTILITY DEFINITIONS
// ===================

// FP_FP_INT(i, frac) -- Create generic f_.frac fixedpoint number
// with integer part i and fractional part 0
#define FP_INT(i, frac)     ((i) << (frac))

// FP_XP(a, b, frac) -- Create generic f_.frac fixedpoint number
// with integer part a and fractional part (b / (2 ** frac))
#define FP_INT_FRAC(a, b, frac)     (((a) << (frac)) | (b))

// FP_FRAC(num, den, frac) -- Create generic f_.frac fixedpoint number
// from the num/den fraction with frac number of bits.
#define FP_FRAC(num, den, frac)     (((num) * (1 << (frac))) / (den) )

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

// FP_MUL3(fpa, fpb, shra, shrb, shrr) -- Do fixedpoint multiplication
// of fixedpoint numbers fpa and fpb. To adjust fractional part of the
// result, shift fpa right by shra, fpb right by shrb and the result
// by shrr.
#define FP_MUL3(fpa, fpb, shra, shrb, shrr)      ((((fpa) >> (shra)) * ((fpb) >> (shrb))) >> (shrr))

// FP_MUL2(fpa, fpb, shra, shrb) -- Do fixedpoint multiplication of 
// fpa and fpb shifting them right by shra and shrb before multiplying
#define FP_MUL2(fpa, fpb, shra, shrb)       FP_MUL3((fpa), (fpb), (shra), (shrb), 0)

// FP_MUL1(fpa, fpb, shrr) -- Do fixedpoint multiplication of fpa and 
// fpb and shift the result right by shrr.
#define FP_MUL1(fpa, fpb, shrr)     FP_MUL3((fpa), (fpb), 0, 0, (shrr))

// FP_EXTEND(fp, fraca, fracb) -- Extend the fractional part of the 
// original fixedpoint number that had fracb bits to have fraca bits.
#define FP_EXTEND(fp, fraca, fracb)     ((fp) << ((fraca) - (fracb)))

// FP_CHUNK(fp, fraca, fracb) -- Chunk the fractional part of the
// original fixedpoint number that had fracb bits to have fraca bits.
#define FP_CHUNK(fp, fraca, fracb)     ((fp) >> ((fracb) - (fraca)))

#ifdef QUADCOPTER
    // Function to to integer square root
    uint32_t fp_sqrt(uint32_t n);
#endif // QUADCOPTER

#endif // FIXEDPOINT_H
