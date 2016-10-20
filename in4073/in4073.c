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

uint32_t iteration = 0;
uint32_t control_iteration = 0;
bool is_test_device = false;

int main(void) {
    init_all();

    profile_start_tag(&qc_state.prof.pr[2], get_time_us(), iteration);
    while (1) {
        if (check_timer_flag()) {
            profile_start_tag(&qc_state.prof.pr[0], get_time_us(), iteration);
            qc_system_step(&qc_system);
            profile_end(&qc_state.prof.pr[0], get_time_us());
            led_display();
            clear_timer_flag();
            control_iteration++;
        }

        if (check_sensor_int_flag()) {
            profile_end(&qc_state.prof.pr[2], get_time_us());
            if (qc_state.option.raw_control) {
                sensor_fifo_count = 1;
                while (sensor_fifo_count) {
                    get_raw_sensor_data();
                    qc_state.sensor.sax =  sax * ACC_G_SCALE_INV - qc_state.offset.sax;
                    qc_state.sensor.say = -say * ACC_G_SCALE_INV - qc_state.offset.say;
                    qc_state.sensor.saz = -saz * ACC_G_SCALE_INV - qc_state.offset.saz;
                    qc_state.sensor.sp  = GYRO_CONV_FROM_NATIVE( sp) - qc_state.offset.sp;
                    qc_state.sensor.sq  = GYRO_CONV_FROM_NATIVE(-sq) - qc_state.offset.sq;
                    qc_state.sensor.sr  = GYRO_CONV_FROM_NATIVE(-sr) - qc_state.offset.sr; 
                    qc_kalman_filter(&qc_state);
                }
            } else {
                sensor_fifo_count = 1;
                while (sensor_fifo_count)
                    get_dmp_data();
                qc_state.sensor.sphi    = FP_MUL3((int32_t)FP_FLOAT(10.f, 10), phi    , 0, 2, 8);
                qc_state.sensor.stheta  = FP_MUL3((int32_t)FP_FLOAT(10.f, 10), theta  , 0, 2, 8);
                qc_state.sensor.spsi    = FP_MUL3((int32_t)FP_FLOAT(10.f, 10), psi    , 0, 2, 8);
                //qc_kalman_filter(&qc_state);
            }
            profile_start_tag(&qc_state.prof.pr[2], get_time_us(), iteration);
            clear_sensor_int_flag();
        }

        if (text_queue.count) {
            transmit_text();
        }

        while (rx_queue.count)
            serialcomm_receive_char(&serialcomm, dequeue(&rx_queue));
        iteration++;
    }
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
