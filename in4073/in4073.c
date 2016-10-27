/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

// Quadcopter flying logic
// =======================

// Variables
// ---------

qc_system_t         qc_system;
qc_mode_table_t     qc_mode_tables[MODE_COUNT];
qc_state_t          qc_state;
qc_command_t        qc_command;
serialcomm_t        serialcomm;
qc_hal_t            qc_hal;

uint32_t led_patterns[] = {0, 0, 0, 0};

// Custom functions
// ----------------
extern void qc_hal_tx_byte(uint8_t);

static void qc_rx_complete(message_t*);
static void init_modes(void);
static void led_display(void);
static void init_all(void);
static void transmit_text(void);
static bool process_and_control(void);
static void receive_commands(void);
static void process_dmp_data(void);
static bool process_raw_data(void);
static void idle_task(bool);

uint32_t iteration = 0;
uint32_t control_iteration = 0;
bool is_test_device = false;

int main(void) {
    init_all();

    while (1) {

        // This is priority round robin scheduling.
        // The higher priority task is the first in the chain of
        // if (...) else if (...) sequences. This guarantees that then
        // latency for the highest priority task is the time needed to
        // complete a single other task.
        bool finished = true;
        if (check_sensor_int_flag() || !finished) {
            idle_task(false);
            clear_sensor_int_flag();
            // Processing the data might happen before all data is read.
            // In this case we want to enter this branch again, that is
            // what the "finished" flag is for.
            finished = process_and_control();
        }
        else if (rx_queue.count) {
            idle_task(false);
            receive_commands();
        }
        else if (check_timer_flag()) {
            clear_timer_flag();
            idle_task(false);
            qc_hal.get_inputs_fn(&qc_state);
            led_display();
            qc_system_log_data(&qc_system);
        }
        else if (text_queue.count) {
            idle_task(false);
            transmit_text();
        }
        else {
            idle_task(true);
        }

        iteration++;
    }
}

bool process_and_control(void) {
    // Start measuring pr0: Time from sensor interrupt until outputs are applied to the motor.
    profile_start_tag(&qc_state.prof.pr[0], get_time_us(), control_iteration);
    // End measuring pr2: Time from applying outputs to new data from sensor.
    profile_end(&qc_state.prof.pr[2], get_time_us());

    // Calculate outputs for the control system according to current mode.
    // ========================
    bool finished;
    if (qc_state.option.raw_control) {
        finished = process_raw_data();
    } else {
        process_dmp_data();
        finished = true;
    }
    qc_system_step(&qc_system);
    // ========================

    // Start measuring pr0: Time from applying outputs until new sensor interrupt arrives.
    profile_start_tag(&qc_state.prof.pr[2], get_time_us(), control_iteration);
    profile_end(&qc_state.prof.pr[0], get_time_us());

    control_iteration++;
    return finished;
}

bool process_raw_data(void) {
    static int iter_count = 0;
    do {
        sensor_fifo_count = 0;
        // Start measuring pr3: Time of one data read
        profile_start_tag(&qc_state.prof.pr[3], get_time_us(), control_iteration);
        get_raw_sensor_data();
        qc_state.sensor.sax =  sax * ACC_G_SCALE_INV - qc_state.offset.sax;
        qc_state.sensor.say = -say * ACC_G_SCALE_INV - qc_state.offset.say;
        qc_state.sensor.saz = -saz * ACC_G_SCALE_INV - qc_state.offset.saz;
        qc_state.sensor.sp  = GYRO_CONV_FROM_NATIVE( sp) - qc_state.offset.sp;
        qc_state.sensor.sq  = GYRO_CONV_FROM_NATIVE(-sq) - qc_state.offset.sq;
        qc_state.sensor.sr  = GYRO_CONV_FROM_NATIVE(-sr) - qc_state.offset.sr;
        acc_filter(&qc_state);
        qc_kalman_filter(&qc_state);
        iter_count = iter_count + 1;
        if (iter_count == 4) iter_count = 0;
        profile_end(&qc_state.prof.pr[3], get_time_us());
    } while (sensor_fifo_count && (iter_count != 3));
    //if ((control_iteration & (0x1F << 3)) == 0)
    //    printf("iter:%d fifo:%d\n", iter_count, sensor_fifo_count);
    return (sensor_fifo_count == 0);
}

void process_dmp_data(void) {
    int iter_count = 0;
    do {
        sensor_fifo_count = 0;
        // Start measuring pr3: Time of one data read
        profile_start_tag(&qc_state.prof.pr[3], get_time_us(), control_iteration);
        get_dmp_data();
        profile_end(&qc_state.prof.pr[3], get_time_us());
        iter_count++;
    } while (sensor_fifo_count);
    //if ((control_iteration & (0x1F << 3)) == 0)
    //    printf("iter:%d fifo:%d\n", iter_count, sensor_fifo_count);

    qc_state.sensor.sax =  sax * ACC_G_SCALE_INV - qc_state.offset.sax;
    qc_state.sensor.say = -say * ACC_G_SCALE_INV - qc_state.offset.say;
    qc_state.sensor.saz = -saz * ACC_G_SCALE_INV - qc_state.offset.saz;
    qc_state.sensor.sp  = GYRO_CONV_FROM_NATIVE( sp) - qc_state.offset.sp;
    qc_state.sensor.sq  = GYRO_CONV_FROM_NATIVE(-sq) - qc_state.offset.sq;
    qc_state.sensor.sr  = GYRO_CONV_FROM_NATIVE(-sr) - qc_state.offset.sr; 
    qc_state.sensor.sphi    = FP_MUL3((int32_t)FP_FLOAT(5.f, 0), phi    , 0, 0, 0) - qc_state.offset.sphi;
    qc_state.sensor.stheta  = FP_MUL3((int32_t)FP_FLOAT(5.f, 0), theta  , 0, 0, 0) - qc_state.offset.stheta;
    qc_state.sensor.spsi    = FP_MUL3((int32_t)FP_FLOAT(5.f, 0), psi    , 0, 0, 0);
    qc_kalman_height(&qc_state);
}

void receive_commands(void) {
    while (rx_queue.count)
        serialcomm_receive_char(&serialcomm, dequeue(&rx_queue));
}

void idle_task(bool is_idle) {
    // This is used only to measure the time when the processor is
    // idle. If this time is very low or especially zero then we have
    // problems because higher priority tasks don't have time to run.
    static bool was_idle = true;
    if (was_idle && !is_idle) {
        profile_end(&qc_state.prof.pr[4], get_time_us()); // End measuring idle time
    } else if (!was_idle && is_idle) {
        profile_start(&qc_state.prof.pr[4], get_time_us()); // Start measuring idle time
    }
    was_idle = is_idle;
}

void init_all(void) {
    is_test_device = NRF_FICR->DEVICEID[0] == TESTDEVICE_ID0 && NRF_FICR->DEVICEID[1] == TESTDEVICE_ID1;
    // Hardware init
    uart_init();
    gpio_init();
    timers_init();
    adc_init();
    twi_init();
    //imu_init(true, 100); <-- initialized in qc_system_init
    baro_init();
    //spi_flash_init(); <-- initialized in log_init
    //ble_init(); 
    // HAL & software init
    qc_hal_init(&qc_hal);
    init_modes();
    qc_system_init(
        &qc_system,
        MODE_0_SAFE,
        qc_mode_tables,
        &qc_state,
        &qc_command,
        &serialcomm,
        &qc_rx_complete,
        &qc_hal
    );
    profile_start_tag(&qc_state.prof.pr[2], get_time_us(), control_iteration);
    profile_start_tag(&qc_state.prof.pr[4], get_time_us(), control_iteration);
    qc_command.timer = qc_hal.get_time_us_fn();
}

void init_modes(void) {
    mode_0_safe_init(&qc_mode_tables[MODE_0_SAFE]);
    mode_1_panic_init(&qc_mode_tables[MODE_1_PANIC]);
    mode_2_manual_init(&qc_mode_tables[MODE_2_MANUAL]);
    mode_3_calibrate_init(&qc_mode_tables[MODE_3_CALIBRATE]);
    mode_4_yaw_init(&qc_mode_tables[MODE_4_YAW]);
    mode_5_full_init(&qc_mode_tables[MODE_5_FULL_CONTROL]);
}

void transmit_text(void) {
    message_value_t msgv;
    msgv.v32[0] = 0;
    msgv.v32[1] = 0;
    for (int i = 0; i < MESSAGE_VALUE_SIZE && text_queue.count; i++) {
        msgv.v8[i] = dequeue(&text_queue);
    }
    serialcomm_quick_send(&serialcomm, MESSAGE_TEXT_ID, msgv.v32[0], msgv.v32[1]);
}

void qc_rx_complete(message_t* message) {
    qc_command_rx_message(&qc_command, message);
}

void led_display(void) {
    static uint32_t counter = 0;
    static const uint32_t colors[] = {BLUE, GREEN, YELLOW, RED};
    //static uint8_t intensities[] = {0, 0, 0, 0};

    led_patterns[3] = 0;
    switch (qc_system.mode) {
        case MODE_0_SAFE:
            led_patterns[1] = 0xffffffff;
            break;
        case MODE_1_PANIC:
            led_patterns[1] = 0;
            led_patterns[3] = 0xffffffff;
            break;
        case MODE_2_MANUAL:
            led_patterns[1] = 0xfafafafa;
            break;
        case MODE_3_CALIBRATE:
            led_patterns[1] = 0xffeaffea;
            break;
        case MODE_4_YAW:
            led_patterns[1] = 0xffaaffaa;
            break;
        case MODE_5_FULL_CONTROL:
            led_patterns[1] = 0xfeaafeaa;
            break;
        default:
            break;
    }
    led_patterns[0] = 0xFF00FF00;
    if (qc_state.prof.pr[4].last_delta < 10) {
        led_patterns[2] = ~0x00000000;
    } else if (qc_state.prof.pr[4].last_delta < 20) {
        led_patterns[2] = ~0x11111111;
    } else if (qc_state.prof.pr[4].last_delta < 50) {
        led_patterns[2] = ~0x33333333;
    } else if (qc_state.prof.pr[4].last_delta < 100) {
        led_patterns[2] = ~0x77777777;
    } else {
        led_patterns[2] = ~0xFFFFFFFF;
    }

    counter++;
    for (int i = 0; i < 4; i++) {
        if ((led_patterns[i] & (1ul << ((counter >> 3) & 0x1F))) == 0) {
            nrf_gpio_pin_set(colors[i]);
        } else {
            nrf_gpio_pin_clear(colors[i]);
        }
    }
    //if ((counter & 0xFF) == 0)
    //    printf("> I'm alive!\n");
}

void Default_Handler(void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void NMI_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void HardFault_Handler (void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_clear(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void SVC_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_clear(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void PendSV_Handler (void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_clear(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void SysTick_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_clear(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

// Start critical section code
// Original code by Boldizsar Palotas for previous university project.
unsigned int CRITICALSECTION_NestingLevel = 0;

// End critical section code
