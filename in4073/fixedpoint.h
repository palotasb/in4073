#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

// Typedef fixedpoint types to give semantic meaning to fixed point variables and values.

// Fixed point unsigned value with 32 bits integer 0 bits fraction
typedef uint32_t    uf32p0;
// Fixed point unsigned value with 24 bits integer 8 bits fraction
typedef uint32_t    uf24p8;
// Fixed point unsigned value with 16 bits integer 16 bits fraction
typedef uint32_t    uf16p16;
// Fixed point unsigned value with 8 bits integer 24 bits fraction
typedef uint32_t    uf8p24;
// Fixed point unsigned value with 0 bits integer 32 bits fraction
typedef uint32_t    uf0p32;

// Fixed point signed value with 32 bits integer 0 bits fraction
typedef int32_t     f32p0;
// Fixed point signed value with 24 bits integer 8 bits fraction
typedef int32_t     f24p8;
// Fixed point signed value with 16 bits integer 16 bits fraction
typedef int32_t     f16p16;
// Fixed point signed value with 8 bits integer 24 bits fraction
typedef int32_t     f8p24;
// Fixed point signed value with 0 bits integer 32 bits fraction
typedef int32_t     f0p32;

// Fixed point unsigned value with 16 bits integer 0 bits fraction
typedef uint16_t    uf16p0;
// Fixed point unsigned value with 8 bits integer 8 bits fraction
typedef uint16_t    uf8p8;
// Fixed point unsigned value with 0 bits integer 16 bits fraction
typedef uint16_t    uf0p16;

// Fixed point signed value with 16 bits integer 0 bits fraction
typedef int16_t     f16p0;
// Fixed point signed value with 8 bits integer 8 bits fraction
typedef int16_t     f8p8;
// Fixed point signed value with 0 bits integer 16 bits fraction
typedef int16_t     f0p16;

// Fixed point unsigned value with 8 bits integer 0 bits fraction
typedef uint8_t     uf8p0;
// Fixed point unsigned value with 0 bits integer 8 bits fraction
typedef uint8_t     uf0p8;

// Fixed point signed value with 8 bits integer 0 bits fraction
typedef int8_t      if8p0;
// Fixed point signed value with 0 bits integer 8 bits fraction
typedef int8_t      if0p8;

#define FP_P0(i)    (i)
#define FP_P8(i)    ((i) << 8)
#define FP_P16(i)   ((i) << 16)
#define FP_P24(i)   ((i) << 24)
#define FP_P32(i)   ((i) << 32)

#define FP_XP8(a, b)    (((a) <<  8) | (b))
#define FP_XP16(a, b)   (((a) << 16) | (b))
#define FP_XP24(a, b)   (((a) << 24) | (b))
#define FP_XP32(a, b)   (((a) << 32) | (b))

#define FP_INT_P0(i)    (i)
#define FP_INT_P8(i)    ((i) >> 8)
#define FP_INT_P16(i)   ((i) >> 16)
#define FP_INT_P24(i)   ((i) >> 24)
#define FP_INT_P32(i)   ((i) >> 32)

#define FP_FRAC_P0(i)   0
#define FP_FRAC_P8(i)   ((i) & 0xFF)
#define FP_FRAC_P16(i)  ((i) & 0xFFFF)
#define FP_FRAC_P24(i)  ((i) & 0xFFFFFF)
#define FP_FRAC_P32(i)  ((i) & 0xFFFFFFFF)

#endif // FIXEDPOINT_H
