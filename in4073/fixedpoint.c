#include "fixedpoint.h"

/** =======================================================
 *  fp_sqrt -- Fast integer or fixedpoint square root.
 *  =======================================================
 *  Does fast integer or fixedpoint square root
 *  calculation.
 *  Input       Output      To get input_type from output
 *  uf0p32_t    uf16p16_t   (output << 16)
 *  uf8p24_t    uf20.12     (output << 12)
 *  uf16p16_t   uf24p8_t    (output << 8)
 *  uf24p8_t    uf28.4      (output << 4)
 *  uf32p0_t    uf32p0_t    no-op
 *  uint32_t    uint32_t    no-op
 *
 *  Parameters:
 *  - n: The input.
 *  Source:
 *  http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2 
**/
uint32_t fp_sqrt(uint32_t n) {
    uint32_t op  = n;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

    // "one" starts at the highest power of four <= than the argument.
    while (op < one)
        one >>= 2;

    while (one != 0) {
        if (res + one <= op) {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }

    // Do arithmetic rounding to nearest integer
    if (res < op)
        res++;

    return res;
}