#ifndef MODE_3_CALIBRATE_H
#define MODE_3_CALIBRATE_H

#include "qc_mode.h"
#include "fixedpoint.h"

/** Calibration mode overview
 *  ==========================
 *
 *  This is a generic overview of the quadcopter operating in calibration mode
 *  
 *  Quadcopter system diagram
 *  -------------------------
 *
 *                                                                         : Plant, or
 *                                                                         : Quadcopter
 *                                                                         :        /
 *                                                                                 /
 *                                                                   +---+ :  +---+
 *                                                                   | 0 |--->| P |
 *                                                                   +---+ :  +---+
 *                                                                         :    | f
 *                   +-------> sp_off                                      :    | t
 *                   |         sa_off                              .......:    V
 *               +---^---+                                         :         +------+
 *        +----->|Average|<-------------------+                    :   ω_n | | c∫dt |
 *        |      +-------+                    |                    :       | +------+
 *        |                                   |                 +------+   V    | v
 *        |                                   |               +-| Gyro |<- O <--+ ω
 *        |                                   |             sp| +------+  ω     V
 *        |                                   +---------------+    :         +------+
 *        |                                                        :   φ_n | | ∫dt  |
 *        +---------------------------------------------------+    :   Z_n | +------+
 *                                                          sa| +------+   V    | x
 *                                                            +-| Acc. |<- O <--+ φ
 *                                                              +------+ [φ;θ;Z]
 *                                                                 :
 *                                              Computed values <- : -> Physical values
 */

typedef struct mode_3_calibrate_state {
    f24p8_t     sp;
    f24p8_t     sq;
    f24p8_t     sr;
    f24p8_t     sax;
    f24p8_t     say;
    f24p8_t     saz;
    f24p8_t     sphi;
    f24p8_t     stheta;
    f24p8_t     pressure;
	 uint8_t	 counter;
	 bool			busy;
} mode_3_calibrate_state_t;


void mode_3_calibrate_init(qc_mode_table_t* mode_table);


#endif // MODE_3_CALIBRATE_H
