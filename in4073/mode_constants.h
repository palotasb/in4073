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

// Inverse of the control loop time constant in seconds
// 1 / 0.01 = 1000 / 10 [1/s] in Q24.8 format.
#define T_INV       ((q32_t)(FP_FRAC(1000, 10, 8)))

// Moment of inertia around the x, y, z axis (for L, M, N torque) [N m]
 // 1/2 [N m] in Q24.8 format.
#define I_L         FP_FRAC(1, 2, 8)
 // 1/2 [N m] in Q24.8 format.
#define I_M         FP_FRAC(1, 2, 8)
 // 1/2 [N m] in Q24.8 format.
#define I_N         FP_FRAC(1, 2, 8)

// Inverse of the product of the control loop time constant and the
// moment of inertia for the L, M, N torque
// Q24.8
#define T_INV_I_L   FP_MUL1(T_INV, I_L, 8)
#define T_INV_I_M   FP_MUL1(T_INV, I_M, 8)
#define T_INV_I_N   FP_MUL1(T_INV, I_N, 8)
// FP_MUL1 is the post-shifted fixp multiplication.

// The b' and d' constants and some commonly-used multiples of them.
#define B           FP_INT(26, 8)
#define D           FP_INT(10, 8)
#define M1_4B       (- B / 4)
#define P1_2B       (B / 2)
#define P1_4D       (D / 4)

// pi in a Q3.29 (highest precision) format
#define PI_Q29      1686629713

// pi/180 in a Q1.31 format
#define PI_180_Q31  37480660

// Convert byte from range [-128; 127] to [-0.56 rad; 0.56 rad]
// That is from -32º to 31.75º
// This can be interpreted as byte value = degrees * 4.
// Going further this can be interpreted that byte is a Q6.2 format
// fixedpoint degree value. From there the conversion:
#define RADIAN_FROM_BYTE(byte)  FP_MUL3((byte), PI_180_Q31, 0, 8, 17)
// (Q6.2) * (8Q1.23) = 8Q7.25

#endif // MODE_CONSTANTS_H
