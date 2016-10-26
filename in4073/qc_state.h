#ifndef QC_STATE_H
#define QC_STATE_H

#include <inttypes.h>
#include <stdbool.h>
#include "fixedpoint.h"
#include "profile.h"

/** State: orientation
 *  Controller setpoint signals
 *  ------------------
 *  Fields:
 *  - lift [m s^-2]: lift acceleration along z axis (downwards)
 *  - roll [rad]: phi angle of rotation around longitudinal x axis
 *  - pitch [rad]: theta angle between quadcopter longitudinal and horizontal
 *  - yaw [rad]: psi angle between Earth frame x axis and Body frame x axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_orient {
    // Q8.8
    q32_t       lift;
    // Q2.14
    q32_t       roll;
    // Q2.14
    q32_t       pitch;
    // Q6.10
    q32_t       yaw;
} qc_state_orient_t;

/** State: motor
 *  Motor speed signals
 *  ------------------
 *  Fields:
 *  - ae1..4 [relative unit]: Motor speed setpoint signal with values 0 to ??? TODO
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_motor {
    uint16_t    ae1;
    uint16_t    ae2;
    uint16_t    ae3;
    uint16_t    ae4;
} qc_state_motor_t;

/** State: sensor
 *  Sensor readings of the quadcopter
 *  ------------------
 *  Fields:
 *  - sp [rad s^-1]: angular speed reading around Body frame x axis
 *  - sq [rad s^-1]: angular speed reading around Body frame y axis
 *  - sr [rad s^-1]: angular speed reading around Body frame z axis
 *  - sax [m s^-2]: acceleration in the Body frame x axis direction
 *  - say [m s^-2]: acceleration in the Body frame y axis direction
 *  - saz [m s^-2]: acceleration in the Body frame z axis direction
 *  - temperature [ºC]
 *  - pressure [mbar]
 *  - battery [V]
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_sensor {
    f16p16_t    sphi;
    f16p16_t    stheta;
    f16p16_t    spsi;
    f16p16_t    sp;
    f16p16_t    sq;
    f16p16_t    sr;
    f16p16_t    sax;
    f16p16_t    say;
    f16p16_t    saz;
    f8p8_t      temperature;
    f16p16_t    pressure;
    f16p16_t    pressure_avg;
    f16p16_t    voltage;
    f16p16_t    voltage_avg;
} qc_state_sensor_t;

/** State: offset
 *  Sensor offsets of the quadcopter
 *  ------------------
 *  Fields:
 *  - sp [rad s^-1]: angular speed offset around Body frame x axis
 *  - sq [rad s^-1]: angular speed offset around Body frame y axis
 *  - sr [rad s^-1]: angular speed offset around Body frame z axis
 *  - sax [m s^-2]: acceleration offset in the Body frame x axis direction
 *  - say [m s^-2]: acceleration offset in the Body frame y axis direction
 *  - saz [m s^-2]: acceleration offset in the Body frame z axis direction
 *  Author: Koos Eerden
**/
typedef struct qc_state_offset {
    f16p16_t    sp;
    f16p16_t    sq;
    f16p16_t    sr;
    f16p16_t    sax;
    f16p16_t    say;
    f16p16_t    saz;
    f16p16_t    sphi;
    f16p16_t    stheta;
    bool        calibrated;
} qc_state_offset_t;

/** State: position
 *  Position of quadcopter in Earth frame
 *  ------------------
 *  Fields:
 *  - x [m]: position along x axis
 *  - y [m]: position along y axis
 *  - z [m]: position along z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_pos {
    f16p16_t    x;
    f16p16_t    y;
    f16p16_t    z;
} qc_state_pos_t;

/** State: attitude
 *  Attitude of quadcopter in Earth frame Euler angles
 *  ------------------
 *  Fields:
 *  - phi [rad]: roll around x axis
 *  - theta [rad]: pitch angle around y axis
 *  - psi [rad]: yaw angle around z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_att {
    f16p16_t    phi;
    f16p16_t    theta;
    f16p16_t    psi;
} qc_state_att_t;

/** State: forces
 *  Forces acting on the quadcopter in the Body frame
 *  ------------------
 *  Fields:
 *  - X [N = kg m s^-2]: force acting along x axis
 *  - Y [N = kg m s^-2]: force acting along y axis
 *  - Z [N = kg m s^-2]: force acting along z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_force {
    f16p16_t    X;
    f16p16_t    Y;
    f16p16_t    Z;
} qc_state_force_t;

/** State: torque
 *  Rotational forces, torque acting on the quadcopter in the body frame
 *  ------------------
 *  Fields:
 *  - L [N m = kg m^2 s^-2]: torque around x axis
 *  - M [N m = kg m^2 s^-2]: torque around y axis
 *  - N [N m = kg m^2 s^-2]: torque around z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_torque {
    f16p16_t    L;
    f16p16_t    M;
    f16p16_t    N;
} qc_state_torque_t;

/** State: velocity
 *  Velocity of the quadcopter in the Body frame
 *  ------------------
 *  Fields:
 *  - u [m s^-1]: velocity along x axis
 *  - v [m s^-1]: velocity along y axis
 *  - w [m s^-1]: velocity along z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_velo {
    f16p16_t    u;
    f16p16_t    v;
    f16p16_t    w;
} qc_state_velo_t;

/** State: spin
 *  Rotational velocity of the quadcopter in the Body frame
 *  ------------------
 *  Fields:
 *  - u [rad s^-1]: rotational velocity around x axis
 *  - v [rad s^-1]: rotational velocity around y axis
 *  - w [rad s^-1]: rotational velocity around z axis
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_spin {
    f16p16_t    p;
    f16p16_t    q;
    f16p16_t    r;
} qc_state_spin_t;

/** State: controller trimming parameters
 *  Trimming parameters for controllers in various modes
 *  ------------------
 *  Fields:
 *  - yaw_p [1]: P controller parameter for yaw control
 *  - p1 [1]: P controller parameter for roll/pitch controller
 *  - p2 [1]: P controller parameter for roll/pitch controller
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_trim {
    f16p16_t    yaw_p;
    f16p16_t    p1;
    f16p16_t    p2;
} qc_state_trim_t;

/** State: options
 *  Options for flying parameters
 *  ------------------
 *  Fields:
 *  - height_control: enable height control with the help of the barometer
 *  - raw_control: enable precision sensor data processing by using custom filtering
 *  - wireless_control: enable untethered flying via wireless Bluetooth LE connection
 *  - enable_motors: enable physical spinning of motors for real unsimulated flying
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_option {
    bool        height_control;
    bool        raw_control;
    bool        wireless_control;
    bool        enable_motors;
} qc_state_option_t;

#define QC_STATE_PROF_CNT   5
/** State: prof
 *  Placeholders for profiling different parts of the system.
 *  ------------------
 *  Fields:
 *  - prN: Profiling information slot N.
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state_prof {
    profile_t   pr[QC_STATE_PROF_CNT];
} qc_state_prof_t;

/** State
 *  
 *  ------------------
 *  Fields:
 *  - orient: Orientation (controller setpoint) information
 *  - motor: Motor speed information
 *  - sensor: Sensor readings
 *  - offset: Sensor offsets after calibration
 *  - pos: Position information (Earth frame)
 *  - att: Attitude information (Earth frame, Euler angles)
 *  - force: Forces (Body frame)
 *  - torque: Torques (Body frame)
 *  - velo: Velocity (Body frame)
 *  - spin: Angular velocity (Body frame)
 *  - trim: Controller trimming parameters
 *  - option: Other quadcopter options
 *  - prof: Profiling information
 *  Author: Boldizsar Palotas
**/
typedef struct qc_state {
    qc_state_orient_t   orient;
    qc_state_motor_t    motor;
    qc_state_sensor_t   sensor;
    qc_state_offset_t   offset;
    qc_state_pos_t      pos;
    qc_state_att_t      att;
    qc_state_force_t    force;
    qc_state_torque_t   torque;
    qc_state_velo_t     velo;
    qc_state_spin_t     spin;
    qc_state_trim_t     trim;
    qc_state_option_t   option;
    qc_state_prof_t     prof;
} qc_state_t;

void qc_state_init(qc_state_t* state);

void qc_state_clear_orient(qc_state_t* state);

void qc_state_clear_motor(qc_state_t* state);

void qc_state_clear_sensor(qc_state_t* state);

void qc_state_clear_offset(qc_state_t* state);

void qc_state_clear_pos(qc_state_t* state);

void qc_state_clear_att(qc_state_t* state);

void qc_state_clear_force(qc_state_t* state);

void qc_state_clear_torque(qc_state_t* state);

void qc_state_clear_velo(qc_state_t* state);

void qc_state_clear_spin(qc_state_t* state);

void qc_state_clear_trim(qc_state_t* state);

void qc_state_clear_option(qc_state_t* state);

void qc_state_clear_prof(qc_state_t* state);

#endif // QC_STATE_H
