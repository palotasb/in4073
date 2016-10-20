#ifndef SIMULATION_H
#define SIMULATION_H

/** Quadcopter simulation
 *  =====================
 *
 *  +-----------------+ pipe:serialout +----------------+
 *  |                 |--------------->| Simulation     |
 *  |   PC terminal   | pipe:serialin  | +------------+ |
 *  |                 |<---------------| | Quadcopter | |
 *  +-----------------+                | +------------+ |
 *    |           |  ^                 | | Sim. HAL   |<---.
 *    |stdout     |  |                 | +------------+ |  |
 *    |    stderr |  |                 +----------------+  |
 *    V           |   \                                    V
 *  +-----------+ |   |                         +------------+
 *  | Telemetry | |   |                         | Quadcopter |
 *  +-----------+ V   | stdin                   | model      |
 *     +-----------+ +----------------+         +------------+
 *     | PC screen | | PC keyboard/js |
 *     +-----------+ +----------------+
**/

// Standard includes
#include <stdbool.h>
#include <inttypes.h>

// Platform includes
//#include <linux.h>

// Quadcopter includes
#include "../qc_mode.h"
#include "../qc_system.h"
#include "../mode_0_safe.h"
#include "../mode_1_panic.h"
#include "../mode_3_calibrate.h"
#include "../mode_5_full.h"

// Simulation-specific functions
// -----------------------------

// Misc.
int init_all(void);
void init_modes(void);

// Timing
bool sim_check_timer_flag(void);
void sim_clear_timer_flag(void);

// Debug output
void sim_display(void);

// Communication
void sim_comm_send_text(void);
int sim_comm_getchar(void);

// Simulation HAL
void qc_hal_init(qc_hal_t*);
void sim_hal_init(qc_hal_t*);

int simulation_printf(const char* fmt, ...);

// Variables
// ---------

extern bool is_test_device;
extern uint32_t led_patterns[];

#endif // SIMULATION_H
