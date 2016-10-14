#ifndef MODE_CONSTANTS_H
#define MODE_CONSTANTS_H

/** DESCRIPTION OF THE OPERATING MODES
 *  ==================================
 *
 *  Generic system overview
 *  =======================
 *
 *  This is a generic overview of the control system in a cascaded
 *  arrangement. In all the operating modes parts of this generic
 *  control system are implemented in various simple ways.
 *  
 *  Quadcopter system diagram
 *  -------------------------
 *
 *  +-----+
 *  | PC  |
 *  +-----+
 *   | | |
 *   | | +-------------------------------------------------+               : Plant, or
 *   | +------------------------+                          |               : Quadcopter
 *   | x_p                  v_p |                      f_p |               :        /
 *   | φ_p   x_e            ω_p |     v_e              t_p |     f_e     u = ae^2  /
 *   V       φ_e +--+  +----+   V     ω_e +--+  +------+   V     t_e +---+ :  +---+
 *   +--> O ---->|C1|->|d/dt|-> O -> O -->|C2|->|k d/dt|-> O -> O -->| T |--->| P |
 *   x_s  ^ -    +--+  +----+    v_s ^ -  +--+  +------+    f_s ^ -  +---+ :  +---+
 *   φ_s  | ^x                   ω_s | ^v                   t_s | ^f       :    | f
 *        | ^φ                       | ^ω                       | ^t       :    | t
 *        |                          |        +-----------------+   .......:    V
 *        |                          |        |                    :         +------+
 *        |                          |        |                    :   ω_n | | c∫dt |
 *        |                          |        |                    :       | +------+
 *        |                          |        |                 +------+   V    | v
 *        |                          |        |               +-| Gyro |<- O <--+ ω
 *        |                          |        |  +--------+ sp| +------+  ω     V
 *        |                          |        +--| Kalman |<--+    :         +------+
 *        |                          +-----------| filter |        :   φ_n | | ∫dt  |
 *        +--------------------------------------|        |<--+    :   Z_n | +------+
 *                                               +--------+ sa| +------+   V    | x
 *                                                            +-| Acc. |<- O <--+ φ
 *                                                              +------+ [φ;θ;Z]
 *                                                                 :
 *                                              Computed values <- : -> Physical values
 *  Variables
 *  =========
 *
 *  All main variables above are actually vectors comprised of the
 *  scalar variables also mentioned in the assignment.
 *
 *  Name            Vector  Scalar values
 *  --------------  ------  -------------
 *  Position        x       [x, y, z]
 *  Attitude        φ       [φ, θ, ψ]
 *  Speed           v       [v, u, w]
 *  Angular speed   ω       [p, q, r]
 *  Force           f       [X, Y, Z]
 *  Torque, moment  t       [L, M, N]
 *  Motor speed     ae      [ae1, ae2, ae3, ae4]
 *  Gyro speed      sp      [sp, sq, sr]
 *  Accelerometer   sa      [sax, say, saz]
 *  Mass and moment
 *  of inertia      k       [m, m, m, I_x, I_y, I_z]
 *  ... Reciprocal  c       [1/m, ...]
 *
 *  The above values are the physical (true) values of the quantities.
 *  Other types of values are described by indices.
 *  
 *  Notation    Meaning
 *  ----------  -------
 *  a           True physical value of a
 *  a_p         PC-provided value for a
 *  a_s         Compted setpoint for a
 *  ^a          Estimated value of a
 *  a_e         Error of a
 *  a_n         Additive noise in the measurement of a
 *
 *  C1 and C2 are the cascaded controllers as recommended by the
 *  assignment. We could use other architectures too, like a
 *  state-space controller.
 *  
 *  System equations
 *  ================
 *
 *  Basic equations describing quadcopter dynamics
 *  ----------------------------------------------
 *
 *  +-  -+         +-     -+ +- -+     +-  -+
 *  | x' |         | t 0 0 | | u |     | x' |
 *  | y' |     =   | 0 t 0 | | v |   + | y' |
 *  | z' |         | 0 0 t | | w |     | z' |
 *  +-  -+[k+1]    +-     -+ +- -+[k]  +-  -+[k]
 *  Here x', y' and z' are in the body frame so they'd have
 *  to be transformed to yield meaningful x, y, z values.
 *  
 *  +- -+         +-     -+ +- -+     +- -+
 *  | φ |         | t 0 0 | | p |     | φ |
 *  | θ |     =   | 0 t 0 | | q |   + | θ |
 *  | ψ |         | 0 0 t | | r |     | ψ |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  Here I think that the reference frame is the same for
 *  both Earth and Body and the equation is usable as is.
 *
 *  +- -+         +-     -+ +- -+     +- -+
 *  | u |       1 | t 0 0 | | X |     | u |
 *  | v |     = - | 0 t 0 | | Y |   + | v |
 *  | w |       m | 0 0 t | | Z |     | w |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  Here m is the weight of the quadcopter.
 *  
 *  +- -+         +-     -+ +- -+     +- -+
 *  | p |       1 | t 0 0 | | L |     | p |
 *  | q |     = - | 0 t 0 | | M |   + | q |
 *  | r |       I | 0 0 t | | N |     | r |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  I is the moment of inertia of the quadcopter.
 *
 *                            +-      -+
 *  +- -+         +-       -+ | ae_1^2 |
 *  | X |         | 0 0 0 0 | | ae_2^2 |
 *  | Y |    = -b'| 0 0 0 0 | | ae_3^2 |
 *  | Z |         | 1 1 1 1 | | ae_4^2 |
 *  +- -+[k+1]    +-       -+ +-      -+[k]
 *  Here b' is a conversion constant.
 *                                   +-      -+
 *  +- -+         +-              -+ | ae_1^2 |
 *  | L |         |  0  -b'  0   b'| | ae_2^2 |
 *  | M |     =   |  b'  0  -b'  0 | | ae_3^2 |
 *  | N |         | -d'  d' -d'  d'| | ae_4^2 |
 *  +- -+[k+1]    +-              -+ +-      -+[k]
 *  Here b' and d' are conversion constants.
 *
 *  Relation to sensor readings (to be used in Kalman filter)
 *  ---------------------------------------------------------
 *  
 *  +- -+       +-     -+ +-  -+
 *  | p |       | 1 0 0 | | sp |
 *  | q |   =   | 0 1 0 | | sq |
 *  | r |       | 0 0 1 | | sr |
 *  +- -+[k]    +-     -+ +-  -+[k]
 *  
 *  +- -+       +-     -+ +-   -+
 *  | φ |       | 0 n 0 | | sax |
 *  | θ |   =   | n 0 0 | | say |
 *  | Z |       | 0 0 n | | saz |
 *  +- -+[k]    +-     -+ +-   -+[k]
 *  n = 1/m' where m' is a weight constant
 *  true at small angles where α ~ sin(α)
 *
**/

#include "fixedpoint.h"

#define LIFT_SHIFT     5
#define LIFT_MULTIPLIER (1 << LIFT_SHIFT)
#define ROLL_SHIFT     0
#define PITCH_SHIFT    0
// This could be 10 based on the LAB-4 tests but it seemed too large
// in offline no-joystick tests.
#define YAW_SHIFT      5

// Inverse of the control loop time constant in seconds
// 1 / (0.01 [s]) = 1000 / 10 [1/s] in Q24.8 format.
#define T_INV       ((q32_t)(FP_FRAC(1000, 10, 8)))

// Moment of inertia around the x, y, z axis (for L, M, N torque) [N m]
 // [N m] in Q24.8 format.
#define I_L         FP_FRAC(1, 32, 8)
 // [N m] in Q24.8 format.
#define I_M         FP_FRAC(1, 32, 8)
 // [N m] in Q24.8 format.
#define I_N         FP_FRAC(1, 64, 8)

// Inverse of the product of the control loop time constant and the
// moment of inertia for the L, M, N torque
// Q24.8
#define T_INV_I_L   FP_MUL1(T_INV, I_L, 8)
#define T_INV_I_M   FP_MUL1(T_INV, I_M, 8)
#define T_INV_I_N   FP_MUL1(T_INV, I_N, 8)
// FP_MUL1 is the post-shifted fixp multiplication.

// The b' and d' constants and some commonly-used multiples of them.
// Dimensions don't really matter we just add two bit fractions to
// make the subsequent divisions integers.
// 1/b'
#define _1_B        FP_INT(50, 2)
// 1/d'
#define _1_D        FP_INT(150, 2)
// - 1/(4b')
#define M1_4B       (- _1_B / 4)
// 1/(2b')
#define _1_2B       (_1_B / 2)
// 1/(4d')
#define _1_4D       (_1_D / 4)

// Value of pi in a Qx.29 format (highest precision, if 3 <= x).
#define PI_Q29      1686629713
//                   \  \  \  \.

// Value of pi/180 in a Qx.36 format (highest precision, x can be -4).
// The values of the integer and missing fractional part are zero.
// This is valid int32_t (not just uint32_t)
#define PI_180_Q36  1199381129
//                   \  \  \  \.

// Value of 180/pi in a Qx.25 format (highest precision, if 7 <= x).
#define _180_PI_Q25 1922527338
//                   \  \  \  \.

// Convert  degrees         to radians.
// Format   [Qx.2]          to [Qy.14]
// x can be up to 9 bits to support -180 to +180 range
// 
// In Q6.2 input format, the range is -32 deg to 31.75 deg
// 16 bit range to give some leeway for a few operations.
#define RADIAN_FROM_DEGREE(deg)  FP_MUL3((deg), PI_180_Q36, 0, 11, 13)
// Qz.14    = (Qx.2 * (Qy.36 >> 11)) >> 13
//          = (Qx.2 * (Qy.25)) >> 13
//          = (Qx+y.27)) >> 13
//          = Qz.14

//accelerometer scale factor: 1 over the amount of bits per G
//Its in F16P16 format, meaning 1 over 16384 (= 0.000061035)
#define ACC_G_SCALE_INV 4

//gyroscope scale factor: 1 over the amount of bits per G
//Its in F16P16 format, meaning 1 over 131 (= 0,007633588)
#define GYRO_G_SCALE_INV 500

#define ATT_SCALE_INV 3

#define MAX_MOTOR_SPEED  600

#define ZERO_LIFT_THRESHOLD (4 * (LIFT_MULTIPLIER))


#define P1_FRAC_BITS    4
#define P1_MAX          FP_INT(100, P1_FRAC_BITS)
#define P1_DEFAULT      (FP_FRAC(10, 1,   P1_FRAC_BITS) + 176)
#define P1_MIN          (-(P1_DEFAULT) + 1)

#define P2_FRAC_BITS    4
#define P2_MAX          FP_INT(100, P2_FRAC_BITS)
#define P2_DEFAULT      (FP_FRAC(1, 2,   P2_FRAC_BITS) + 127)
#define P2_MIN          (-(P2_DEFAULT) + 1)

#define YAWP_FRAC_BITS  7
#define YAWP_MAX        FP_INT(10, YAWP_FRAC_BITS)
#define YAWP_DEFAULT    (FP_FRAC(1, 64,   YAWP_FRAC_BITS) + 4)
#define YAWP_MIN        (-(YAWP_DEFAULT) + 1)

#endif // MODE_CONSTANTS_H
