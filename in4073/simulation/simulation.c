#include "simulation.h"
#include "model.h"
#include <stdio.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>

model_t             model;

qc_system_t         qc_system;
qc_mode_table_t     qc_mode_tables[MODE_COUNT];
qc_state_t          qc_state;
qc_command_t        qc_command;
serialcomm_t        serialcomm;
qc_hal_t            sim_hal;

bool                is_test_device;
uint32_t            led_patterns[] = {0, 0, 0, 0};
bool                enable_motors;
uint32_t            iteration;

int fifo_to_term;
int fifo_to_sim;

// String buffer
#define STRBUFF_SIZE 1024
int strbuff_idx = 0;
char strbuff[STRBUFF_SIZE] = {0};

// Log buffer 
#define LOGBUFF_SIZE (1024*1024/8)
uint8_t logbuff[LOGBUFF_SIZE] = {0};

// Local static functions
// ----------------------

// Simulation HAL
static void sim_tx_byte_fn(uint8_t);
static void sim_get_inputs_fn(qc_state_t*);
static void sim_set_outputs_fn(qc_state_t*);
static void sim_enable_motors_fn(bool);
static void sim_rx_complete(message_t*);

static bool sim_flash_init(void);
static bool sim_flash_read(uint32_t, uint8_t*, uint32_t);
static bool sim_flash_write(uint32_t, uint8_t*, uint32_t);
static bool sim_flash_erase(void);

static void sim_void(void);

static int init_fifos(void);

static uint32_t time_get_us(void);

bool timer_tick = false;
unsigned long long timer_last_tick;

int main(void) {
    int i;
    if ((i = init_all())) {
        fprintf(stderr, "Error initalizing.\n");
        return -1;
    }

    fprintf(stderr, "Starting simulation.\n");

    while (1) {
        if (sim_check_timer_flag()) {
            qc_system_step(&qc_system);
            sim_display();
            sim_clear_timer_flag();
        }

        sim_comm_send_text();

        int c;
        while (0 <= (c = sim_comm_getchar()))
            serialcomm_receive_char(&serialcomm, c);
    }
}

// Simulation-specific functions
int init_all(void) {
    is_test_device = true;
    qc_hal_init(&sim_hal);
    init_modes();
    model_init(&model);
    int i;
    if ((i = init_fifos())) {
        if (i == -1) {
            fprintf(stderr, "Error creating FIFO to PC.\n");
            return -1;
        }
    }
    timer_last_tick = time_get_us();
    qc_system_init(
        &qc_system,
        MODE_0_SAFE,
        qc_mode_tables,
        &qc_state,
        &qc_command,
        &serialcomm,
        &sim_rx_complete,
        &sim_hal
    );
    return 0;
}

void init_modes(void) {
    mode_0_safe_init(&qc_mode_tables[MODE_0_SAFE]);
    mode_1_panic_init(&qc_mode_tables[MODE_1_PANIC]);
    mode_2_manual_init(&qc_mode_tables[MODE_2_MANUAL]);
    mode_3_calibrate_init(&qc_mode_tables[MODE_3_CALIBRATE]);
    mode_4_yaw_init(&qc_mode_tables[MODE_4_YAW]);
    mode_5_full_init(&qc_mode_tables[MODE_5_FULL_CONTROL]);
}

// Timing
bool sim_check_timer_flag(void) {
    unsigned long long t_us = time_get_us();
    if (10000 <= t_us - timer_last_tick) {
        timer_tick = true;
        timer_last_tick = t_us;
        return true;
    } else {
        return false;
    }
}

void sim_clear_timer_flag(void) {
    timer_tick = false;
}

// Debug output
void sim_display(void) {
    // TODO display debug information
}

// Communication
int init_fifos(void) {
    errno = 0;
    if (mkfifo("/tmp/fifo_to_term", S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)) {
        if (errno != EEXIST) {
            fprintf(stderr, "Error %d creting fifo to term. (%s)\n", errno, strerror(errno));
            return -1;
        }
    }
    errno = 0;
    if (mkfifo("/tmp/fifo_to_sim", S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)) {
        if (errno != EEXIST) {
            fprintf(stderr, "Error %d creting fifo to sim. (%s)\n", errno, strerror(errno));
            return -1;
        }
    }

    errno = 0;
    if ((fifo_to_sim = open("/tmp/fifo_to_sim", O_RDONLY | O_NONBLOCK)) == -1) {
        fprintf(stderr, "Error %d opening fifo to sim. (%s)", errno, strerror(errno));
        return -2;
    }
    errno = 0;
    if ((fifo_to_term = open("/tmp/fifo_to_term", O_WRONLY)) == -1) {
        fprintf(stderr, "Error %d opening fifo to term. (%s)", errno, strerror(errno));
        return -2;
    }
    return 0;
}

int simulation_printf(const char* fmt, ...) {
    int i;
    va_list argptr;
    va_start(argptr, fmt);
    i = vsnprintf(&strbuff[strbuff_idx], STRBUFF_SIZE - strbuff_idx, fmt, argptr);
    va_end(argptr);
    //va_start(argptr, fmt);
    //vfprintf(stderr, fmt, argptr);
    //va_end(argptr);
    if (0 < i)
        strbuff_idx += i;
    return i;
}

void sim_comm_send_text(void) {
    message_value_t msgv;
    int i = 0;
    while (strbuff[i] && i != STRBUFF_SIZE) {
        msgv.v32[0] = 0;
        msgv.v32[1] = 0;
        int j = 8;
        while (j-- && strbuff[i] && i != STRBUFF_SIZE) {
            msgv.v8[i & 0x7] = strbuff[i];
            i++;
        }
        serialcomm_quick_send(&serialcomm, MESSAGE_TEXT_ID, msgv.v32[0], msgv.v32[1]);
    }
    for (i = 0; i != STRBUFF_SIZE; i++) {
        strbuff[i] = 0;
    }
    strbuff_idx = 0;
}

int sim_comm_getchar(void) {
    unsigned char c;
    int r;
    if ((r = read(fifo_to_sim, &c, 1)) == 1) {
        //fprintf(stderr, "> %c (%d)\n", isprint(c) ? c : ' ', c);
        return c; // Successful read
    } else if (r == 0 ) {
            return -2; // EOF
    } else if (r == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return -1; // OK
        }
        fprintf(stderr, "Getchar error %d. (%s)\n", errno, strerror(errno));
    }
    return -3; // OTHER
}

// Simulation HAL
void qc_hal_init(qc_hal_t* hal) {
    hal->enable_motors_fn = sim_enable_motors_fn;
    hal->get_inputs_fn = sim_get_inputs_fn;
    hal->set_outputs_fn = sim_set_outputs_fn;
    hal->tx_byte_fn = sim_tx_byte_fn;

    hal->flash_init_fn = sim_flash_init;
    hal->flash_read_fn = sim_flash_read;
    hal->flash_write_fn = sim_flash_write;
    hal->flash_erase_fn = sim_flash_erase;

    hal->reset_fn = sim_void;
    hal->get_time_us_fn = time_get_us;
}

void sim_tx_byte_fn(uint8_t byte) {
    write(fifo_to_term, &byte, 1);
}

void sim_get_inputs_fn(qc_state_t* state) {
    state->sensor.voltage = 1100;
    state->sensor.pressure = 100;
    state->sensor.temperature = 100;
    state->sensor.sax = (int32_t)(model.ax * 256 * 256);
    state->sensor.say = (int32_t)(model.ay * 256 * 256);
    state->sensor.saz = (int32_t)(model.az * 256 * 256);
    state->sensor.sp = (int32_t)(model.p * 256 * 256);
    state->sensor.sq = (int32_t)(model.q * 256 * 256);
    state->sensor.sr = (int32_t)(model.r * 256 * 256);
}

void sim_set_outputs_fn(qc_state_t* state) {
    if (enable_motors) {
        model.ae1sq = state->motor.ae1 * state->motor.ae1;
        model.ae2sq = state->motor.ae2 * state->motor.ae2;
        model.ae3sq = state->motor.ae3 * state->motor.ae3;
        model.ae4sq = state->motor.ae4 * state->motor.ae4;
    } else {
        model.ae1sq = 0;
        model.ae2sq = 0;
        model.ae3sq = 0;
        model.ae4sq = 0;
    }
}

void sim_enable_motors_fn(bool enable) {
    enable_motors = enable;
}

void sim_rx_complete(message_t* message) {
    qc_command_rx_message(&qc_command, message);
}

uint32_t time_get_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint32_t)(ts.tv_sec * 1000000ull + ts.tv_nsec / 1000ull);
}

bool sim_flash_init(void) {
    return true;
}

bool sim_flash_read(uint32_t addr, uint8_t* buf, uint32_t size) {
    while (size--) {
        *buf++ = logbuff[addr++];
    }
    return true;
}

bool sim_flash_write(uint32_t addr, uint8_t* buf, uint32_t size) {
    while (size--) {
        logbuff[addr++] = *buf++;
    }
    return true;
}

bool sim_flash_erase(void) {
    return true;
}

void sim_void(void) {}