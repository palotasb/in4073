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
#define YAW_SHIFT      2


// Control loop time constant in seconds
// 0.01s in Q16.16 format
#define T_CONST_FRAC_BITS       10
#define T_CONST                 ((q32_t) FP_FLOAT(0.01, T_CONST_FRAC_BITS))

//minimal (absolute) Z force for which motors are turning. This is used by height-control 
//its in f16p16_t format
//TODO: tune this
#define MIN_Z_FORCE  ((q32_t)FP_FLOAT(12, 16))

// This determines the amount of pressure samples that are averaged in order to filter the pressure.
// please note that if this value equals N, the number of samples equals 2^N
#define PRESSURE_AVERAGE_SHIFT    4 

// Inverse of the control loop time constant in seconds
// 1 / (0.01 [s]) = 1000 / 10 [1/s] in Q24.8 format.
#define T_INV_FRAC_BITS         0
#define T_INV                   ((q32_t)(FP_FRAC(1000, 10, T_INV_FRAC_BITS)))

// Moment of inertia around the x, y, z axis (for L, M, N torque) [N m]
// [N m] in Q24.8 format.
#define I_L         FP_FRAC(1, 32, 8)
// [N m] in Q24.8 format.
#define I_M         FP_FRAC(1, 32, 8)
// [N m] in Q24.8 format.
#define I_N_FRAC_BITS   8
#define I_N             ((int32_t)FP_FLOAT(0.25, I_N_FRAC_BITS))

// Inverse of the product of the control loop time constant and the
// moment of inertia for the L, M, N torque
// Q24.8
#define T_INV_I_L   FP_MUL1(T_INV, I_L, 8)
#define T_INV_I_M   FP_MUL1(T_INV, I_M, 8)
#define T_INV_I_N   FP_MUL1(T_INV, I_N, I_N_FRAC_BITS)
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

// Value of pi/2 in a Qx.30 format (highest precision if 3 <= x)
#define PI_2_Q30    PI_Q29

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

//barometer scale factor: 1 over 100
//Its in F16P16 format
#define BARO_SCALE_INV  FP_FRAC(1, 100, 16)

//accelerometer scale factor: 1 over the amount of bits per G
//Its in F16P16 format, meaning 1 over 16384 (= 0.000061035)
#define ACC_G_SCALE_INV 4

// Gyroscope scale factor:
// FS_native = +/- 2000 DPS = +/- 34.906 rad/s = 69.813 rad/s
// precn_native = FS / (2^16 bit) = 0,001065 (rad/s)/bit
// precn_target = (1 rad/s) / (2^16 bit)
//
// repr_native [bits] * precn_native [rad/s/bit] = value [rad/s]
//
//                 repr_native [bits] * precn_target [(rad/s)/bit]
// value [rad/s] = -----------------------------------------------
//                          precn_native [(rad/s)/bits]
//
// repr_target = repr_native * GYRO_CONV_CONST
// repr_target = repr_native * 69.81317

#define GYRO_CONV_CONST_FRAC_BITS       10
#define GYRO_CONV_CONST                 ((f16p16_t) FP_FLOAT(69.81317, GYRO_CONV_CONST_FRAC_BITS))
#define GYRO_CONV_FROM_NATIVE(value)    FP_MUL1((int32_t)(value) , GYRO_CONV_CONST, GYRO_CONV_CONST_FRAC_BITS)

#define ATT_SCALE_INV 3

#define MAX_MOTOR_SPEED  750

#define ZERO_LIFT_THRESHOLD (4 * (LIFT_MULTIPLIER))


#define P1_FRAC_BITS    0
#define P1_MAX          FP_INT(200, P1_FRAC_BITS)
#define P1_DEFAULT      ((int32_t) FP_FLOAT(40.f, P1_FRAC_BITS) - 28)
#define P1_MIN          (-(P1_DEFAULT) + 1)

#define P2_FRAC_BITS    2
#define P2_MAX          FP_INT(50, P2_FRAC_BITS)
#define P2_DEFAULT      ((int32_t) FP_FLOAT(8.0f, P2_FRAC_BITS) + 100)
#define P2_MIN          (-(P2_DEFAULT) + 1)

#define YAWP_FRAC_BITS  10
#define YAWP_MAX        FP_INT(10, YAWP_FRAC_BITS)
#define YAWP_DEFAULT    ((int32_t) FP_FLOAT(0.035f, YAWP_FRAC_BITS) + 24)
#define YAWP_MIN        (-(YAWP_DEFAULT) + 1)


// TODO tune this value
//Height control P value, its in F8P8 format
#define P_HEIGHT        ((q32_t)FP_FLOAT(100.0, 8))

// Kalman filter constants

#define KALMAN_WEIGHT_FRAC_BITS      12
#define KALMAN_GYRO_WEIGHT      ((q32_t) FP_FLOAT(.99f, KALMAN_WEIGHT_FRAC_BITS))
#define KALMAN_ACC_WEIGHT       (FP_INT(1, KALMAN_WEIGHT_FRAC_BITS) - KALMAN_GYRO_WEIGHT)

// Magic constant 0.6f is needed becaus gyro and accelerometer don't agree on the angle.
#define KALMAN_M_FRAC_BITS      10
#define KALMAN_M                ((int32_t) FP_FLOAT(0.6f * 3.141592f / 2, KALMAN_M_FRAC_BITS))

// IMU constants
#define IMU_RAW_FREQ        1000
#define T_CONST_RAW         FP_MUL1(T_CONST , FP_FRAC(T_INV >> T_INV_FRAC_BITS, IMU_RAW_FREQ, 8), 8)

#endif // MODE_CONSTANTS_H
